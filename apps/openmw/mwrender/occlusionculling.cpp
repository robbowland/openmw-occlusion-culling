#include "occlusionculling.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

#include <osg/AlphaFunc>
#include <osg/BoundingBox>
#include <osg/BoundingSphere>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/NodeVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/Transform>
#include <osgUtil/CullVisitor>
#include <osgUtil/RenderStage>

#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>

#include <components/debug/debuglog.hpp>
#include <components/misc/constants.hpp>
#include <components/sceneutil/occlusionculling.hpp>
#include <components/settings/values.hpp>
#include <components/terrain/terrainoccluder.hpp>

#include "vismask.hpp"

namespace
{
    class CollectMeshVisitor : public osg::NodeVisitor
    {
    public:
        CollectMeshVisitor()
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
        {
        }

        void apply(osg::Transform& transform) override
        {
            osg::Matrix matrix;
            if (!mMatrixStack.empty())
                matrix = mMatrixStack.back();
            transform.computeLocalToWorldMatrix(matrix, this);
            mMatrixStack.push_back(matrix);
            traverse(transform);
            mMatrixStack.pop_back();
        }

        void apply(osg::Drawable& drawable) override
        {
            auto* geom = drawable.asGeometry();
            if (!geom)
                return;

            // Skip alpha-tested drawables (e.g. tree leaf billboards) —
            // their transparent regions would cause false occlusion.
            // NIF loader applies AlphaFunc to the parent Group, not the Geometry itself,
            // so check both the drawable and its immediate parent.
            if (hasAlphaTest(drawable))
                return;

            const auto* verts = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray());
            if (!verts || verts->empty())
                return;

            osg::Matrix matrix;
            if (!mMatrixStack.empty())
                matrix = mMatrixStack.back();

            unsigned int baseVertex = static_cast<unsigned int>(mVertices.size());
            for (const auto& v : *verts)
                mVertices.push_back(v * matrix);

            for (unsigned int p = 0; p < geom->getNumPrimitiveSets(); ++p)
                collectTriangles(geom->getPrimitiveSet(p), baseVertex);
        }

        std::vector<osg::Vec3f> mVertices;
        std::vector<unsigned int> mIndices;

    private:
        static bool hasAlphaTest(const osg::Node& node)
        {
            if (const auto* ss = node.getStateSet())
                if (ss->getAttribute(osg::StateAttribute::ALPHAFUNC))
                    return true;
            for (const auto* parent : node.getParents())
                if (const auto* ss = parent->getStateSet())
                    if (ss->getAttribute(osg::StateAttribute::ALPHAFUNC))
                        return true;
            return false;
        }

        void collectTriangles(const osg::PrimitiveSet* pset, unsigned int baseVertex)
        {
            unsigned int count = pset->getNumIndices();
            switch (pset->getMode())
            {
                case GL_TRIANGLES:
                    for (unsigned int i = 0; i + 2 < count; i += 3)
                    {
                        mIndices.push_back(baseVertex + pset->index(i));
                        mIndices.push_back(baseVertex + pset->index(i + 1));
                        mIndices.push_back(baseVertex + pset->index(i + 2));
                    }
                    break;
                case GL_TRIANGLE_STRIP:
                    for (unsigned int i = 0; i + 2 < count; ++i)
                    {
                        if (i % 2 == 0)
                        {
                            mIndices.push_back(baseVertex + pset->index(i));
                            mIndices.push_back(baseVertex + pset->index(i + 1));
                            mIndices.push_back(baseVertex + pset->index(i + 2));
                        }
                        else
                        {
                            mIndices.push_back(baseVertex + pset->index(i + 1));
                            mIndices.push_back(baseVertex + pset->index(i));
                            mIndices.push_back(baseVertex + pset->index(i + 2));
                        }
                    }
                    break;
                case GL_TRIANGLE_FAN:
                    for (unsigned int i = 1; i + 1 < count; ++i)
                    {
                        mIndices.push_back(baseVertex + pset->index(0));
                        mIndices.push_back(baseVertex + pset->index(i));
                        mIndices.push_back(baseVertex + pset->index(i + 1));
                    }
                    break;
                default:
                    break;
            }
        }

        std::vector<osg::Matrix> mMatrixStack;
    };
}


namespace
{
    // Diagnostic (OPENMW_DRAW_CENSUS=1): classify the main camera's draw calls by the visibility
    // mask of their nearest categorised ancestor, logged every 120 frames.
    struct DrawCensus
    {
        struct Entry
        {
            unsigned int draws = 0;
            unsigned long long vertices = 0;
        };
        std::map<std::string, Entry> byCategory;
        std::vector<std::string> samples;

        static std::string categoryOf(const osg::Drawable* drawable)
        {
            static const std::pair<unsigned int, const char*> masks[] = {
                { MWRender::Mask_Actor, "actor" }, { MWRender::Mask_Player, "player" },
                { MWRender::Mask_Groundcover, "groundcover" }, { MWRender::Mask_Terrain, "terrain" },
                { MWRender::Mask_Static, "static" }, { MWRender::Mask_Object, "object" },
                { MWRender::Mask_Water, "water" }, { MWRender::Mask_SimpleWater, "water" },
                { MWRender::Mask_Sky, "sky" }, { MWRender::Mask_Effect, "effect" },
                { MWRender::Mask_WeatherParticles, "weather" }, { MWRender::Mask_ParticleSystem, "particles" },
            };
            const osg::Node* node = drawable;
            bool paged = false;
            while (node)
            {
                if (node->getName() == "Chunk" || node->getName().rfind("Chunk", 0) == 0)
                    paged = true;
                for (const auto& [mask, name] : masks)
                    if (node->getNodeMask() == mask || (node->getNodeMask() != ~0u && (node->getNodeMask() & mask)))
                        return paged ? std::string(name) + "(paged)" : std::string(name);
                node = node->getNumParents() > 0 ? node->getParent(0) : nullptr;
            }
            return paged ? "other(paged)" : "other";
        }

        void addStateGraph(const osgUtil::StateGraph* graph)
        {
            for (const auto& leaf : graph->_leaves)
            {
                const osg::Drawable* drawable = leaf->getDrawable();
                if (!drawable)
                    continue;
                const std::string category = categoryOf(drawable);
                if (category == "other" && samples.size() < 6)
                {
                    std::string chain;
                    const osg::Node* node = drawable;
                    for (int depth = 0; node && depth < 9; ++depth)
                    {
                        char mask[16];
                        std::snprintf(mask, sizeof(mask), "%x", node->getNodeMask());
                        chain += std::string(node->className()) + ":" + node->getName() + ":" + mask + " < ";
                        node = node->getNumParents() > 0 ? node->getParent(0) : nullptr;
                    }
                    samples.push_back(chain);
                }
                Entry& entry = byCategory[category];
                ++entry.draws;
                if (const osg::Geometry* geometry = drawable->asGeometry())
                    if (const osg::Array* vertices = geometry->getVertexArray())
                        entry.vertices += vertices->getNumElements();
            }
            for (const auto& child : graph->_children)
                addStateGraph(child.second.get());
        }

        void addBin(const osgUtil::RenderBin* bin)
        {
            for (const auto& graph : bin->getStateGraphList())
                addStateGraph(graph);
            for (const auto& leaf : bin->getRenderLeafList())
            {
                if (const osg::Drawable* drawable = leaf->getDrawable())
                    ++byCategory[categoryOf(drawable)].draws;
            }
            for (const auto& child : bin->getRenderBinList())
                addBin(child.second.get());
        }
    };

    bool drawCensusEnabled()
    {
        static const bool enabled = std::getenv("OPENMW_DRAW_CENSUS") != nullptr;
        return enabled;
    }
}

namespace MWRender
{
    OccluderMesh buildSimplifiedMesh(osg::Node* node, int gridRes, float shrinkFactor)
    {
        OccluderMesh mesh;

        CollectMeshVisitor cmv;
        node->accept(cmv);

        if (cmv.mIndices.empty() || cmv.mVertices.size() < 3)
        {
            osg::ComputeBoundsVisitor cbv;
            node->accept(cbv);
            mesh.aabb = cbv.getBoundingBox();
            return mesh;
        }

        for (const auto& v : cmv.mVertices)
            mesh.aabb.expandBy(v);
        const unsigned int res = static_cast<unsigned int>(gridRes);
        float dx = mesh.aabb.xMax() - mesh.aabb.xMin();
        float dy = mesh.aabb.yMax() - mesh.aabb.yMin();
        float dz = mesh.aabb.zMax() - mesh.aabb.zMin();
        float maxDim = std::max({ dx, dy, dz });
        float cellSize = maxDim / res;

        if (cellSize > 0)
        {
            unsigned int resX = std::max(1u, static_cast<unsigned int>(std::ceil(dx / cellSize)));
            unsigned int resY = std::max(1u, static_cast<unsigned int>(std::ceil(dy / cellSize)));

            struct CellData
            {
                osg::Vec3f sum;
                unsigned int count = 0;
                unsigned int newIndex = 0;
            };
            std::unordered_map<unsigned int, CellData> cells;
            std::vector<unsigned int> vertexRemap(cmv.mVertices.size());

            for (size_t i = 0; i < cmv.mVertices.size(); ++i)
            {
                const osg::Vec3f& v = cmv.mVertices[i];
                float fx = (v.x() - mesh.aabb.xMin()) / cellSize;
                float fy = (v.y() - mesh.aabb.yMin()) / cellSize;
                float fz = (v.z() - mesh.aabb.zMin()) / cellSize;
                unsigned int gx = std::min(static_cast<unsigned int>(std::max(fx, 0.0f)), resX - 1);
                unsigned int gy = std::min(static_cast<unsigned int>(std::max(fy, 0.0f)), resY - 1);
                unsigned int gz = std::min(static_cast<unsigned int>(std::max(fz, 0.0f)), res - 1);
                unsigned int cellId = gx + gy * resX + gz * resX * resY;

                auto& cell = cells[cellId];
                cell.sum += v;
                cell.count++;
                vertexRemap[i] = cellId;
            }

            unsigned int nextIdx = 0;
            for (auto& [id, cell] : cells)
            {
                cell.newIndex = nextIdx++;
                mesh.vertices.push_back(cell.sum / static_cast<float>(cell.count));
            }

            std::unordered_set<uint64_t> seen;
            for (size_t i = 0; i + 2 < cmv.mIndices.size(); i += 3)
            {
                unsigned int a = cells[vertexRemap[cmv.mIndices[i]]].newIndex;
                unsigned int b = cells[vertexRemap[cmv.mIndices[i + 1]]].newIndex;
                unsigned int c = cells[vertexRemap[cmv.mIndices[i + 2]]].newIndex;

                // Reject degenerate triangles
                if (a == b || b == c || a == c)
                    continue;

                // Canonical form: rotate min index to front, then ensure consistent order
                unsigned int tri[3] = { a, b, c };
                if (tri[1] < tri[0] && tri[1] < tri[2])
                    std::rotate(tri, tri + 1, tri + 3);
                else if (tri[2] < tri[0] && tri[2] < tri[1])
                    std::rotate(tri, tri + 2, tri + 3);
                if (tri[1] > tri[2])
                    std::swap(tri[1], tri[2]);

                uint64_t key = (uint64_t(tri[0]) << 42) | (uint64_t(tri[1]) << 21) | uint64_t(tri[2]);
                if (seen.insert(key).second)
                {
                    mesh.indices.push_back(a);
                    mesh.indices.push_back(b);
                    mesh.indices.push_back(c);
                }
            }
        }

        if (!mesh.vertices.empty())
        {
            osg::Vec3f center(0, 0, 0);
            for (const auto& v : mesh.vertices)
                center += v;
            center /= static_cast<float>(mesh.vertices.size());
            for (auto& v : mesh.vertices)
                v = center + (v - center) * shrinkFactor;
        }

        return mesh;
    }

    SceneOcclusionCallback::SceneOcclusionCallback(SceneUtil::OcclusionCuller* culler,
        Terrain::TerrainOccluder* occluder, int radiusCells, bool enableTerrainOccluder, bool enableDebugOverlay,
        bool enableDebugMessages, bool enableInteriors)
        : mCuller(culler)
        , mTerrainOccluder(occluder)
        , mRadiusCells(radiusCells)
        , mEnableTerrainOccluder(enableTerrainOccluder)
        , mEnableDebugOverlay(enableDebugOverlay)
        , mEnableDebugMessages(enableDebugMessages)
        , mEnableInteriors(enableInteriors)
    {
    }

    void SceneOcclusionCallback::setCellType(bool isInterior, bool isQuasiExterior)
    {
        mIsInterior = isInterior;
        mIsQuasiExterior = isQuasiExterior;
    }

    void SceneOcclusionCallback::setupDebugOverlay()
    {
        unsigned int w, h;
        mCuller->getResolution(w, h);
        if (w == 0 || h == 0)
            return;

        mDepthPixels.resize(w * h);

        // Create image to hold depth data (luminance float -> converted to RGBA)
        mDebugImage = new osg::Image;
        mDebugImage->allocateImage(w, h, 1, GL_LUMINANCE, GL_FLOAT);

        // Create texture from image
        mDebugTexture = new osg::Texture2D(mDebugImage);
        mDebugTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        mDebugTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
        mDebugTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        mDebugTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        mDebugTexture->setResizeNonPowerOfTwoHint(false);

        // Create POST_RENDER camera in corner of screen
        mDebugCamera = new osg::Camera;
        mDebugCamera->setName("OcclusionDebugCamera");
        mDebugCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        mDebugCamera->setRenderOrder(osg::Camera::POST_RENDER, 100);
        mDebugCamera->setAllowEventFocus(false);
        mDebugCamera->setClearMask(0);
        mDebugCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, 1, 0, 1));
        mDebugCamera->setViewMatrix(osg::Matrix::identity());
        mDebugCamera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
        mDebugCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        mDebugCamera->setCullingActive(false);

        // Scale viewport to show in bottom-left corner (400px wide, aspect-correct height)
        float displayWidth = 400.0f;
        float displayHeight = displayWidth * static_cast<float>(h) / static_cast<float>(w);
        mDebugCamera->setViewport(0, 0, static_cast<int>(displayWidth), static_cast<int>(displayHeight));

        // Create textured quad
        osg::ref_ptr<osg::Geometry> quad
            = osg::createTexturedQuadGeometry(osg::Vec3(0, 0, 0), osg::Vec3(1, 0, 0), osg::Vec3(0, 1, 0));
        quad->setCullingActive(false);

        osg::StateSet* ss = quad->getOrCreateStateSet();
        ss->setTextureAttributeAndModes(0, mDebugTexture, osg::StateAttribute::ON);

        mDebugCamera->addChild(quad);
    }

    void SceneOcclusionCallback::updateDebugOverlay(osgUtil::CullVisitor* cv)
    {
        if (!mDebugCamera)
            return;

        unsigned int w, h;
        mCuller->getResolution(w, h);

        // Read depth buffer from MOC
        mCuller->computePixelDepthBuffer(mDepthPixels.data());

        // Copy to image (normalize: MOC stores 1/w, so closer = larger values)
        float* imageData = reinterpret_cast<float*>(mDebugImage->data());
        for (unsigned int i = 0; i < w * h; ++i)
        {
            float d = mDepthPixels[i];
            // MOC depth is 1/w (reciprocal clip-space w). 0 = far/empty, larger = closer.
            // Clamp and invert for visualization: dark = far, bright = near
            imageData[i] = std::min(d * 50.0f, 1.0f);
        }
        mDebugImage->dirty();

        // Inject debug camera into the cull visitor so it gets rendered
        unsigned int traversalMask = cv->getTraversalMask();
        cv->setTraversalMask(0xffffffff);
        mDebugCamera->accept(*cv);
        cv->setTraversalMask(traversalMask);
    }

    void SceneOcclusionCallback::operator()(osg::Node* node, osgUtil::CullVisitor* cv)
    {
        // Only run occlusion for the main scene camera.
        // Skip shadow cameras, water reflection, and any other cameras.
        osg::Camera* cam = cv->getCurrentCamera();
        if (cam->getName() != Constants::SceneCamera)
        {
            traverse(node, cv);
            return;
        }

        // The scene is traversed multiple times per frame: once for the main cull pass,
        // and again by MWShadowTechnique::cullShadowReceivingScene (same camera name).
        // Only set up MOC on the first traversal; subsequent passes just traverse normally.
        unsigned int frameNumber = cv->getFrameStamp()->getFrameNumber();
        if (frameNumber == mLastFrameNumber)
        {
            traverse(node, cv);
            return;
        }
        mLastFrameNumber = frameNumber;

        // Skip MSOC entirely in interiors (unless enabled via setting)
        if (mIsInterior && !mEnableInteriors)
        {
            traverse(node, cv);
            return;
        }

        // Begin occlusion frame with camera matrices
        mCuller->beginFrame(cam->getViewMatrix(), cam->getProjectionMatrix());

        // Build and rasterize terrain occluder mesh (skip for quasi-exteriors and interiors — no real terrain)
        if (mEnableTerrainOccluder && !mIsQuasiExterior && !mIsInterior && mTerrainOccluder->hasTerrainData())
        {
            mPositions.clear();
            mIndices.clear();
            mTerrainOccluder->build(cv->getEyePoint(), mRadiusCells, mPositions, mIndices);

            if (!mPositions.empty())
                mCuller->rasterizeTerrainOccluder(mPositions, mIndices);
        }

        // Continue normal cull traversal — CellOcclusionCallbacks will test against the buffer
        traverse(node, cv);

        // End the occlusion frame so sub-camera traversals (water reflection/refraction,
        // shadow cameras) that share this scene graph don't incorrectly cull against
        // the main camera's occlusion buffer.
        if (drawCensusEnabled() && cv->getFrameStamp()->getFrameNumber() % 120 == 0)
        {
            DrawCensus census;
            census.addBin(cv->getCurrentRenderStage());
            std::string line = "DRAW_CENSUS";
            for (const auto& [category, entry] : census.byCategory)
                line += " " + category + "=" + std::to_string(entry.draws) + "/" + std::to_string(entry.vertices);
            Log(Debug::Info) << line;
            for (const auto& sample : census.samples)
                Log(Debug::Info) << "DRAW_CENSUS_OTHER " << sample;
        }

        mCuller->endFrame();

        // Update debug overlay AFTER traversal (terrain + building occluders now in buffer)
        if (mEnableDebugOverlay)
        {
            if (!mDebugCamera)
                setupDebugOverlay();
            updateDebugOverlay(cv);
        }

        if (mEnableDebugMessages)
        {
            static int frameCount = 0;
            if (++frameCount % 300 == 0)
            {
                const auto terrainTris = mIndices.size() / 3;
                const auto bldgTris = mCuller->getNumBuildingTris();
                const auto terrainVerts = mPositions.size();
                const auto bldgVerts = mCuller->getNumBuildingVerts();
                Log(Debug::Info) << "OcclusionCull: terrain tris=" << terrainTris << " terrain verts=" << terrainVerts
                                 << " bldg occluders=" << mCuller->getNumBuildingOccluders()
                                 << " bldg tris=" << bldgTris << " bldg verts=" << bldgVerts
                                 << " total tris=" << (terrainTris + bldgTris)
                                 << " total verts=" << (terrainVerts + bldgVerts)
                                 << " tested=" << mCuller->getNumTested()
                                 << " occluded=" << mCuller->getNumOccluded();
            }
        }
    }

    PagedOccluderCallback::PagedOccluderCallback(
        SceneUtil::OcclusionCuller* culler, float maxDistance, unsigned int maxTriangles)
        : mCuller(culler)
        , mMaxDistanceSq(maxDistance * maxDistance)
        , mMaxTriangles(maxTriangles)
    {
    }

    void PagedOccluderCallback::operator()(osg::Node* node, osgUtil::CullVisitor* cv)
    {
        if (!mCuller->isFrameActive())
        {
            traverse(node, cv);
            return;
        }

        // Transform chunk bounding sphere from local to world space.
        // The chunk sits under a PAT, so node->getBound() is in chunk-local space.
        const osg::BoundingSphere& bs = node->getBound();
        if (bs.valid())
        {
            osg::Matrixd viewInverse;
            viewInverse.invert(cv->getCurrentCamera()->getViewMatrix());
            const osg::Matrixd modelToWorld = *cv->getModelViewMatrix() * viewInverse;
            const osg::Vec3f worldCenter = bs.center() * modelToWorld;
            const float r = bs.radius();

            osg::BoundingBox worldBB(worldCenter.x() - r, worldCenter.y() - r, worldCenter.z() - r, worldCenter.x() + r,
                worldCenter.y() + r, worldCenter.z() + r);

            // If entire chunk is occluded, skip rasterization AND traversal
            if (!mCuller->testVisibleAABB(worldBB))
                return;

            // Rasterize nearby building occluder meshes for visible chunks
            const osg::Vec3f eyeWorld(viewInverse(3, 0), viewInverse(3, 1), viewInverse(3, 2));

            if (auto* udc = node->getUserDataContainer())
            {
                for (unsigned int i = 0; i < udc->getNumUserObjects(); ++i)
                {
                    if (auto* pod = dynamic_cast<PagedOccluderData*>(udc->getUserObject(i)))
                    {
                        for (const auto& occMesh : pod->mOccluderMeshes)
                        {
                            if (occMesh.indices.empty())
                                continue;

                            const osg::Vec3f center = occMesh.aabb.center();
                            if ((center - eyeWorld).length2() > mMaxDistanceSq)
                                continue;

                            unsigned int newTris = static_cast<unsigned int>(occMesh.indices.size() / 3);
                            if (mMaxTriangles > 0 && mCuller->getNumBuildingTris() + newTris > mMaxTriangles)
                                continue;

                            mCuller->rasterizeOccluder(occMesh.vertices, occMesh.indices);
                            mCuller->incrementBuildingOccluders(newTris,
                                static_cast<unsigned int>(occMesh.vertices.size()));
                        }
                        break;
                    }
                }
            }
        }

        traverse(node, cv);
    }

    CellOcclusionCallback::CellOcclusionCallback(SceneUtil::OcclusionCuller* culler, float occluderMinRadius,
        float occluderMaxRadius, float occluderShrinkFactor, int occluderMeshResolution, int occluderMaxMeshResolution,
        float occluderInsideThreshold, float occluderMaxDistance, bool enableStaticOccluders,
        unsigned int maxTriangles)
        : mCuller(culler)
        , mOccluderMinRadius(occluderMinRadius)
        , mOccluderMaxRadius(occluderMaxRadius)
        , mOccluderShrinkFactor(occluderShrinkFactor)
        , mOccluderMeshResolution(occluderMeshResolution)
        , mOccluderMaxMeshResolution(occluderMaxMeshResolution)
        , mOccluderInsideThreshold(occluderInsideThreshold)
        , mOccluderMaxDistanceSq(occluderMaxDistance * occluderMaxDistance)
        , mEnableStaticOccluders(enableStaticOccluders)
        , mMaxTriangles(maxTriangles)
        , mCullLargeObjects(Settings::camera().mOcclusionCullLargeObjects)
    {
    }

    namespace
    {
        const osg::PositionAttitudeTransform* asPat(const osg::Node* node)
        {
            const osg::Transform* transform = node->asTransform();
            return transform ? transform->asPositionAttitudeTransform() : nullptr;
        }
    }

    const OccluderMesh* CellOcclusionCallback::getOccluderMesh(osg::Node* node)
    {
        const osg::PositionAttitudeTransform* pat = asPat(node);
        auto it = mMeshCache.find(node);
        if (it != mMeshCache.end())
        {
            CachedMesh& cached = it->second;
            if (cached.mNode.get() == node)
            {
                // Meshes are world-space; an object that has moved since (e.g. an animated boat)
                // would leave its occluder behind, so it stops being an occluder altogether.
                if (!cached.mMoved && pat
                    && (pat->getPosition() != cached.mPosition || pat->getAttitude() != cached.mAttitude))
                    cached.mMoved = true;
                return cached.mMoved ? nullptr : &cached.mMesh;
            }
            mMeshCache.erase(it); // freed node, address reused by a new object
        }
        if (!mCuller->consumeMeshBuild())
            return nullptr;

        int meshRes = mOccluderMeshResolution;
        float radius = node->getBound().radius();
        if (radius > mOccluderMinRadius && mOccluderMinRadius > 0)
        {
            float scale = radius / mOccluderMinRadius;
            meshRes = std::clamp(
                static_cast<int>(mOccluderMeshResolution * scale), mOccluderMeshResolution, mOccluderMaxMeshResolution);
        }
        OccluderMesh mesh = buildSimplifiedMesh(node, meshRes, mOccluderShrinkFactor);

        if (mesh.indices.empty() && !mesh.aabb.valid())
        {
            Log(Debug::Verbose) << "OccMesh cached (no triangles): \"" << node->getName() << "\"";
        }
        else
        {
            Log(Debug::Verbose) << "OccMesh cached: \"" << node->getName() << "\" verts=" << mesh.vertices.size()
                                << " tris=" << (mesh.indices.size() / 3) << " sphere=" << node->getBound().radius();
        }

        CachedMesh cached;
        cached.mNode = node;
        if (pat)
        {
            cached.mPosition = pat->getPosition();
            cached.mAttitude = pat->getAttitude();
        }
        cached.mMesh = std::move(mesh);
        return &mMeshCache.insert_or_assign(node, std::move(cached)).first->second.mMesh;
    }

    void CellOcclusionCallback::rasterizeLargeOccluder(
        osg::Node* child, const OccluderMesh& mesh, osgUtil::CullVisitor* cv)
    {
        // Rasterize as occluder if in range and camera is not inside the building.
        // Test against terrain-only buffer so other buildings don't prevent rasterization
        // of adjacent buildings (which would reduce culling coverage for Pass 2).
        if (!mesh.aabb.valid() || !mEnableStaticOccluders || mesh.indices.empty()
            || !mCuller->testVisibleAABBTerrainOnly(mesh.aabb))
            return;
        const osg::BoundingSphere& bs = child->getBound();
        if ((bs.center() - cv->getEyePoint()).length2() >= mOccluderMaxDistanceSq)
            return;
        osg::Vec3f center = mesh.aabb.center();
        osg::Vec3f halfExtent
            = (osg::Vec3f(mesh.aabb.xMax(), mesh.aabb.yMax(), mesh.aabb.zMax()) - center) * mOccluderInsideThreshold;
        osg::BoundingBox scaledBB;
        scaledBB.expandBy(center - halfExtent);
        scaledBB.expandBy(center + halfExtent);
        if (scaledBB.contains(cv->getEyePoint()))
            return;
        unsigned int newTris = static_cast<unsigned int>(mesh.indices.size() / 3);
        if (mMaxTriangles == 0 || mCuller->getNumBuildingTris() + newTris <= mMaxTriangles)
        {
            mCuller->rasterizeOccluder(mesh.vertices, mesh.indices);
            mCuller->incrementBuildingOccluders(newTris, static_cast<unsigned int>(mesh.vertices.size()));
        }
    }

    void CellOcclusionCallback::operator()(osg::Group* node, osgUtil::CullVisitor* cv)
    {
        // If occlusion is not active this frame (interior, shadow camera, etc.), traverse normally
        if (!mCuller->isFrameActive())
        {
            traverse(node, cv);
            return;
        }

        // Test cell bounding box against terrain-only depth — if fully hidden by terrain,
        // skip entire cell. Use terrain-only so buildings in adjacent cells don't
        // false-cull entire cells that are clearly in view.
        const osg::BoundingSphere& cellBS = node->getBound();
        if (cellBS.valid())
        {
            osg::BoundingBox cellBB;
            cellBB.expandBy(cellBS);

            if (!mCuller->testVisibleAABBTerrainOnly(cellBB))
                return; // Entire cell hidden by terrain — no children traversed
        }

        const unsigned int numChildren = node->getNumChildren();

        // Pass 1: Large objects — test against terrain depth, optionally rasterize as occluders
        for (unsigned int i = 0; i < numChildren; ++i)
        {
            osg::Node* child = node->getChild(i);
            const osg::BoundingSphere& bs = child->getBound();

            if (!bs.valid() || bs.radius() < mOccluderMinRadius)
                continue;

            // Paged chunks and other oversized objects — test visibility, rasterize stored occluders
            if (bs.radius() > mOccluderMaxRadius)
            {
                // Rasterize sub-object occluder meshes stored at chunk creation time
                if (mEnableStaticOccluders)
                {
                    if (auto* udc = child->getUserDataContainer())
                    {
                        for (unsigned int j = 0; j < udc->getNumUserObjects(); ++j)
                        {
                            if (auto* pod = dynamic_cast<PagedOccluderData*>(udc->getUserObject(j)))
                            {
                                for (const auto& occMesh : pod->mOccluderMeshes)
                                {
                                    if (occMesh.indices.empty())
                                        continue;

                                    unsigned int newTris = static_cast<unsigned int>(occMesh.indices.size() / 3);
                                    if (mMaxTriangles > 0
                                        && mCuller->getNumBuildingTris() + newTris > mMaxTriangles)
                                        continue;

                                    mCuller->rasterizeOccluder(occMesh.vertices, occMesh.indices);
                                    mCuller->incrementBuildingOccluders(
                                        newTris, static_cast<unsigned int>(occMesh.vertices.size()));
                                }
                                break; // Only one PagedOccluderData per chunk
                            }
                        }
                    }
                }

                // Test chunk visibility against terrain-only depth — paged chunks are large
                // geometry that should only be culled by terrain, not adjacent buildings.
                osg::BoundingBox pageBB;
                pageBB.expandBy(bs);
                if (mCullLargeObjects ? mCuller->testVisibleAABB(pageBB) : mCuller->testVisibleAABBTerrainOnly(pageBB))
                    child->accept(*cv);
                continue;
            }

            // Get cached occluder mesh (with AABB for visibility test)
            const OccluderMesh* mesh = getOccluderMesh(child);
            if (mesh)
                rasterizeLargeOccluder(child, *mesh, cv);

            if (mCullLargeObjects)
                continue; // traversed after every large occluder in this cell is rasterized

            // Always traverse large buildings. Do NOT gate traversal on testVisibleAABB —
            // buildings testing against a buffer that includes previously rasterized
            // buildings causes false culling (flickering) when child ordering happens to
            // place one building in front of another in the depth buffer. Large buildings
            // are correctly culled by PVS and the cell-level AABB test above; MSOC
            // is reserved for culling small objects in Pass 2.
            child->accept(*cv);
        }

        // Pass 1b (occlusion cull large objects): every large occluder in the cell is now in the
        // buffer, so test each large object against it. An object cannot hide itself (the test
        // uses its nearest depth), but a simplified mesh can close an arch or doorway.
        if (mCullLargeObjects)
        {
            for (unsigned int i = 0; i < numChildren; ++i)
            {
                osg::Node* child = node->getChild(i);
                const osg::BoundingSphere& bs = child->getBound();
                if (!bs.valid() || bs.radius() < mOccluderMinRadius || bs.radius() > mOccluderMaxRadius)
                    continue;
                osg::BoundingBox childBB;
                childBB.expandBy(bs);
                if (mCuller->testVisibleAABB(childBB))
                    child->accept(*cv);
            }
        }

        // Pass 2: Small objects — test against enriched depth buffer (terrain + buildings)
        for (unsigned int i = 0; i < numChildren; ++i)
        {
            osg::Node* child = node->getChild(i);
            const osg::BoundingSphere& bs = child->getBound();

            if (!bs.valid())
            {
                child->accept(*cv);
                continue;
            }

            if (bs.radius() >= mOccluderMinRadius)
                continue; // Already handled in pass 1

            // Never occlude doors — they sit flush against building surfaces
            // and are easily falsely hidden by the parent building's AABB occluder
            bool skipOcclusion = false;
            child->getUserValue("skipOcclusion", skipOcclusion);

            osg::BoundingBox childBB;
            childBB.expandBy(bs);

            if (skipOcclusion || mCuller->testVisibleAABB(childBB))
                child->accept(*cv);
            // else: occluded — skip
        }
    }
}
