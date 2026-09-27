#include "creatureanimation.hpp"

#include <osg/MatrixTransform>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadcrea.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/sceneutil/lightcommon.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/visitor.hpp>
#include <components/settings/values.hpp>

#include "../mwmechanics/weapontype.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "../mwworld/cellstore.hpp"
#include "../mwworld/class.hpp"

namespace MWRender
{
    namespace
    {
        SceneUtil::CreatureSlope::Shape slopeShape()
        {
            switch (Settings::game().mActorCollisionShapeType)
            {
                case DetourNavigator::CollisionShapeType::RotatingBox:
                    return SceneUtil::CreatureSlope::Shape::RotatingBox;
                case DetourNavigator::CollisionShapeType::Cylinder:
                    return SceneUtil::CreatureSlope::Shape::Cylinder;
                case DetourNavigator::CollisionShapeType::Aabb:
                    break;
            }
            return SceneUtil::CreatureSlope::Shape::Aabb;
        }
    }

    CreatureAnimation::CreatureAnimation(
        const MWWorld::Ptr& ptr, const std::string& model, Resource::ResourceSystem* resourceSystem, bool animated)
        : ActorAnimation(ptr, osg::ref_ptr<osg::Group>(ptr.getRefData().getBaseNode()), resourceSystem)
    {
        MWWorld::LiveCellRef<ESM::Creature>* ref = mPtr.get<ESM::Creature>();

        if (!model.empty())
        {
            if (Settings::game().mCreatureSlopeAlignment
                && SceneUtil::CreatureSlope::eligible(ref->mBase->mFlags & ESM::Creature::Walks,
                    ref->mBase->mFlags & ESM::Creature::Bipedal, ref->mBase->mFlags & ESM::Creature::Flies))
            {
                mSlopeParent = mInsert;
                mSlopeTransform = new osg::MatrixTransform;
                mSlopeTransform->setDataVariance(osg::Object::DYNAMIC);
                mSlopeParent->addChild(mSlopeTransform);
                mInsert = mSlopeTransform;
                mPreviousSlopePosition = ptr.getRefData().getPosition().asVec3();
                mSlopeDebug = Settings::game().mCreatureSlopeDebug;
            }
            setObjectRoot(model, false, false, true);

            if ((ref->mBase->mFlags & ESM::Creature::Bipedal))
                addAnimSource(Settings::models().mXbaseanim.get(), model);

            if (animated)
                addAnimSource(model, model);
        }
    }

    CreatureAnimation::~CreatureAnimation()
    {
        removeFromScene();
    }

    void CreatureAnimation::removeFromScene()
    {
        ActorAnimation::removeFromScene();
        if (mSlopeParent && mSlopeTransform)
            mSlopeParent->removeChild(mSlopeTransform);
    }

    osg::Vec3f CreatureAnimation::runAnimation(float duration)
    {
        // Root motion remains in model coordinates, independent of visual tilt.
        const osg::Vec3f movement = ActorAnimation::runAnimation(duration);
        if (mSlopeTransform)
        {
            const auto& position = mPtr.getRefData().getPosition();
            const osg::Vec3f origin = position.asVec3();
            if ((origin - mPreviousSlopePosition).length2() > 256.f * 256.f)
            {
                mSlope.reset();
                mSlopeSinkResidual = 0.f;
            }
            mPreviousSlopePosition = origin;
            const osg::Quat yaw(position.rot[2], osg::Vec3f(0.f, 0.f, -1.f));
            MWBase::World* world = MWBase::Environment::get().getWorld();

            osg::Vec3f normal(0.f, 0.f, 1.f);
            float sink = 0.f;
            bool onTerrain = false;
            if (mSlopeParams.mEnabled)
            {
                const std::optional<osg::Vec3f> contact = world->getActorVisualGroundNormal(mPtr);
                const osg::Vec3f halfExtents = world->getHalfExtents(mPtr);
                const MWWorld::CellStore* cell = mPtr.getCell();
                if (contact && cell->isExterior())
                {
                    // Terrain: fit the footprint and measure the real gap under the origin. Both are
                    // continuous while walking, unlike the contact triangle and the hull's step height.
                    const ESM::RefId worldspace = cell->getCell()->getWorldSpace();
                    const float sinYaw = std::sin(position.rot[2]);
                    const float cosYaw = std::cos(position.rot[2]);
                    const osg::Vec3f forward(sinYaw, cosYaw, 0.f);
                    const osg::Vec3f right(cosYaw, -sinYaw, 0.f);
                    const float halfLength = std::max(halfExtents.y(), 8.f);
                    const float halfWidth = std::max(halfExtents.x(), 8.f);
                    auto height = [&](const osg::Vec3f& point) { return world->getTerrainHeightAt(point, worldspace); };
                    const float gap = origin.z() - height(origin);
                    const osg::Vec3f fitted = SceneUtil::CreatureSlope::footprintNormal(forward, right, halfLength,
                        halfWidth, height(origin + forward * halfLength), height(origin - forward * halfLength),
                        height(origin + right * halfWidth), height(origin - right * halfWidth));
                    // Standing on terrain rather than a static above it: the origin is no higher than
                    // the upright hull can rest on this plane.
                    const float bound
                        = SceneUtil::CreatureSlope::restingGap(halfExtents, slopeShape(), yaw, fitted) + 16.f;
                    if (gap > -8.f && gap < bound)
                    {
                        onTerrain = true;
                        normal = fitted;
                        // The physics hull keeps a fixed ground offset even on flat ground.
                        sink = mSlopeParams.mSink * std::max(0.f, gap - 1.f);
                    }
                }
                if (!onTerrain && contact)
                    normal = *contact;
            }
            const osg::Quat tilt = mSlope.update(normal, yaw, duration, mSlopeParams);
            mSlopeNormal = normal;
            if (!onTerrain && mSlopeParams.mSink > 0.f)
                sink = mSlope.sinkOffset(world->getHalfExtents(mPtr), slopeShape(), yaw, mSlopeParams);
            // Changing between the terrain and contact sources blends instead of popping.
            if (onTerrain != mSlopeOnTerrain)
                mSlopeSinkResidual = mSlopeSink - sink;
            mSlopeOnTerrain = onTerrain;
            if (std::isfinite(duration) && duration > 0.f)
                mSlopeSinkResidual *= std::exp(-15.f * std::min(duration, 0.1f));
            mSlopeSink = sink + mSlopeSinkResidual;
            mSlopeTransform->setMatrix(osg::Matrix::rotate(tilt) * osg::Matrix::translate(0.f, 0.f, -mSlopeSink));
            if (mSlopeDebug && duration > 0.f)
            {
                mSlopeLogTimer += duration;
                if (mSlopeLogTimer >= 1.f)
                {
                    mSlopeLogTimer = 0.f;
                    const osg::Vec3f renderedUp = yaw * (tilt * osg::Vec3f(0.f, 0.f, 1.f));
                    Log(Debug::Info) << "SLOPE_POSE id=" << mPtr.getCellRef().getRefId()
                                     << " pos=" << position.pos[0] << "," << position.pos[1] << "," << position.pos[2]
                                     << " normal=" << normal.x() << "," << normal.y() << "," << normal.z()
                                     << " renderedUp=" << renderedUp.x() << "," << renderedUp.y() << ","
                                     << renderedUp.z() << " sink=" << mSlopeSink << " terrain=" << onTerrain;
                }
            }
        }
        return movement;
    }

    std::optional<Animation::GroundAlignmentState> CreatureAnimation::getGroundAlignment() const
    {
        if (!mSlopeTransform)
            return std::nullopt;
        return GroundAlignmentState{ mSlopeParams, mSlopeNormal, mSlope.worldUp(), mSlopeSink, mSlopeOnTerrain };
    }

    bool CreatureAnimation::setGroundAlignment(const SceneUtil::CreatureSlope::Params& params)
    {
        if (!mSlopeTransform)
            return false;
        mSlopeParams = params;
        return true;
    }

    CreatureWeaponAnimation::CreatureWeaponAnimation(
        const MWWorld::Ptr& ptr, const std::string& model, Resource::ResourceSystem* resourceSystem, bool animated)
        : ActorAnimation(ptr, osg::ref_ptr<osg::Group>(ptr.getRefData().getBaseNode()), resourceSystem)
        , mShowWeapons(false)
        , mShowCarriedLeft(false)
    {
        MWWorld::LiveCellRef<ESM::Creature>* ref = mPtr.get<ESM::Creature>();

        if (!model.empty())
        {
            setObjectRoot(model, true, false, true);

            if ((ref->mBase->mFlags & ESM::Creature::Bipedal))
                addAnimSource(Settings::models().mXbaseanim.get(), model);

            if (animated)
                addAnimSource(model, model);

            mPtr.getClass().getInventoryStore(mPtr).setInvListener(this);

            updateParts();
        }

        mWeaponAnimationTime = std::make_shared<WeaponAnimationTime>(this);
    }

    void CreatureWeaponAnimation::showWeapons(bool showWeapon)
    {
        if (showWeapon != mShowWeapons)
        {
            mShowWeapons = showWeapon;
            updateParts();
        }
    }

    void CreatureWeaponAnimation::showCarriedLeft(bool show)
    {
        if (show != mShowCarriedLeft)
        {
            mShowCarriedLeft = show;
            updateParts();
        }
    }

    void CreatureWeaponAnimation::updateParts()
    {
        mAmmunition.reset();
        mWeapon.reset();
        mShield.reset();

        updateHolsteredWeapon(!mShowWeapons);
        updateQuiver();
        updateHolsteredShield(mShowCarriedLeft);

        if (mShowWeapons)
            updatePart(mWeapon, MWWorld::InventoryStore::Slot_CarriedRight);
        if (mShowCarriedLeft)
            updatePart(mShield, MWWorld::InventoryStore::Slot_CarriedLeft);
    }

    void CreatureWeaponAnimation::updatePart(PartHolderPtr& scene, int slot)
    {
        if (!mObjectRoot)
            return;

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator it = inv.getSlot(slot);

        if (it == inv.end())
        {
            scene.reset();
            return;
        }
        MWWorld::ConstPtr item = *it;

        std::string_view bonename;
        VFS::Path::Normalized itemModel = item.getClass().getCorrectedModel(item);
        if (slot == MWWorld::InventoryStore::Slot_CarriedRight)
        {
            if (item.getType() == ESM::Weapon::sRecordId)
            {
                const ESM::RefId type = item.get<ESM::Weapon>()->mBase->mData.mType;
                bonename = MWMechanics::getWeaponType(type)->mAttachBone;
                if (bonename != "Weapon Bone")
                {
                    const NodeMap& nodeMap = getNodeMap();
                    NodeMap::const_iterator found = nodeMap.find(bonename);
                    if (found == nodeMap.end())
                        bonename = "Weapon Bone";
                }
            }
            else
                bonename = "Weapon Bone";
        }
        else
        {
            bonename = "Shield Bone";
            if (item.getType() == ESM::Armor::sRecordId)
            {
                itemModel = getShieldMesh(item, false);
            }
        }

        try
        {
            osg::ref_ptr<osg::Node> attached
                = attach(itemModel, bonename, bonename, item.getType() == ESM::Light::sRecordId);

            scene = std::make_unique<PartHolder>(attached);

            if (!item.getClass().getEnchantment(item).empty())
                mGlowUpdater
                    = SceneUtil::addEnchantedGlow(attached, mResourceSystem, item.getClass().getEnchantmentColor(item));

            // Crossbows start out with a bolt attached
            // FIXME: code duplicated from NpcAnimation
            if (slot == MWWorld::InventoryStore::Slot_CarriedRight && item.getType() == ESM::Weapon::sRecordId
                && item.get<ESM::Weapon>()->mBase->mData.mType == ESM::WeaponType::MarksmanCrossbow)
            {
                const ESM::WeaponType* weaponInfo = MWMechanics::getWeaponType(ESM::WeaponType::MarksmanCrossbow);
                MWWorld::ConstContainerStoreIterator ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
                if (ammo != inv.end() && ammo->get<ESM::Weapon>()->mBase->mData.mType == weaponInfo->mAmmoType)
                    attachArrow();
                else
                    mAmmunition.reset();
            }
            else
                mAmmunition.reset();

            std::shared_ptr<SceneUtil::ControllerSource> source;

            if (slot == MWWorld::InventoryStore::Slot_CarriedRight)
                source = mWeaponAnimationTime;
            else
                source = mAnimationTimePtr[0];

            SceneUtil::AssignControllerSourcesVisitor assignVisitor(std::move(source));
            attached->accept(assignVisitor);

            if (item.getType() == ESM::Light::sRecordId)
                addExtraLight(scene->getNode()->asGroup(), SceneUtil::LightCommon(*item.get<ESM::Light>()->mBase));
        }
        catch (std::exception& e)
        {
            Log(Debug::Error) << "Can not add creature part: " << e.what();
        }
    }

    bool CreatureWeaponAnimation::isArrowAttached() const
    {
        return mAmmunition != nullptr;
    }

    void CreatureWeaponAnimation::detachArrow()
    {
        WeaponAnimation::detachArrow(mPtr);
        updateQuiver();
    }

    void CreatureWeaponAnimation::attachArrow()
    {
        WeaponAnimation::attachArrow(mPtr);

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
        if (ammo != inv.end() && !ammo->getClass().getEnchantment(*ammo).empty())
        {
            osg::Group* bone = getArrowBone();
            if (bone != nullptr && bone->getNumChildren())
                SceneUtil::addEnchantedGlow(
                    bone->getChild(0), mResourceSystem, ammo->getClass().getEnchantmentColor(*ammo));
        }

        updateQuiver();
    }

    void CreatureWeaponAnimation::releaseArrow(float attackStrength, float attackWindUp)
    {
        WeaponAnimation::releaseArrow(mPtr, attackStrength, attackWindUp);
        updateQuiver();
    }

    osg::Group* CreatureWeaponAnimation::getArrowBone()
    {
        if (!mWeapon)
            return nullptr;

        if (!mPtr.getClass().hasInventoryStore(mPtr))
            return nullptr;

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return nullptr;

        const ESM::RefId type = weapon->get<ESM::Weapon>()->mBase->mData.mType;
        const ESM::RefId ammoType = MWMechanics::getWeaponType(type)->mAmmoType;
        if (ammoType.empty())
            return nullptr;

        // Try to find and attachment bone in actor's skeleton, otherwise fall back to the ArrowBone in weapon's mesh
        osg::Group* bone = getBoneByName(MWMechanics::getWeaponType(ammoType)->mAttachBone);
        if (bone == nullptr)
        {
            SceneUtil::FindByNameVisitor findVisitor("ArrowBone");
            mWeapon->getNode()->accept(findVisitor);
            bone = findVisitor.mFoundNode;
        }
        return bone;
    }

    osg::Node* CreatureWeaponAnimation::getWeaponNode()
    {
        return mWeapon ? mWeapon->getNode().get() : nullptr;
    }

    Resource::ResourceSystem* CreatureWeaponAnimation::getResourceSystem()
    {
        return mResourceSystem;
    }

    void CreatureWeaponAnimation::addControllers()
    {
        Animation::addControllers();
        if (mObjectRoot)
            WeaponAnimation::addControllers(mNodeMap, mActiveControllers, mObjectRoot.get());
    }

    osg::Vec3f CreatureWeaponAnimation::runAnimation(float duration)
    {
        osg::Vec3f ret = Animation::runAnimation(duration);

        WeaponAnimation::configureControllers(mPtr.getRefData().getPosition().rot[0] + getBodyPitchRadians());

        return ret;
    }

}
