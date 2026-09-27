#ifndef OPENMW_COMPONENTS_SCENEUTIL_CREATURESLOPE_H
#define OPENMW_COMPONENTS_SCENEUTIL_CREATURESLOPE_H

#include <osg/Quat>
#include <osg/Vec3f>

#include <algorithm>
#include <cmath>
#include <utility>

namespace SceneUtil
{
    // Per-actor tuning, exposed to Lua through openmw.animation. Defaults reproduce the
    // original fixed behaviour: 25 degree lean cap, 45 degree support limit, rate 10, no sink.
    // Corpses lie flat against the ground, so they may lean further (35 degrees).
    // Declared outside CreatureSlope so it can be a complete type in default arguments.
    struct CreatureSlopeParams
    {
        bool mEnabled = true;
        float mMaxLean = 0.43633231f; // radians
        float mMinSupportZ = 0.70710678f; // cosine of the steepest accepted support normal
        float mResponsiveness = 10.f; // exponential approach rate, per second
        float mSink = 0.f; // fraction of the collision-shape slope gap to lower the model by
        float mCorpseMaxLean = 0.61086524f; // radians; replaces mMaxLean once the actor is dead
    };

    // Dropped and placed items: a one-off, saved rotation onto the surface they land on.
    // Controlled from Lua (openmw.world.setItemGroundAlignment); off unless a script enables it.
    struct ItemSlopeParams
    {
        bool mEnabled = false;
        float mMaxTilt = 0.52359878f; // radians; steeper surfaces are capped to this tilt
        float mMinSupportZ = 0.70710678f; // cosine; surfaces steeper than 45 degrees leave items upright
    };

    // Visual-only pose. Smooth in world space so turning does not drag the slope around.
    class CreatureSlope
    {
    public:
        using Params = CreatureSlopeParams;

        enum class Shape
        {
            Aabb,
            RotatingBox,
            Cylinder,
        };

        // Amphibious walkers may align on land; world contact validation rejects swimming.
        static constexpr bool eligible(bool walks, bool bipedal, bool flies)
        {
            return walks && !bipedal && !flies;
        }

        static osg::Quat target(osg::Vec3f normal, const Params& params = {})
        {
            if (!params.mEnabled)
                return {};
            if (!std::isfinite(normal.x()) || !std::isfinite(normal.y()) || !std::isfinite(normal.z())
                || normal.length2() < 0.0001f)
                return {};
            normal.normalize();
            // Reject walls and extreme edge contacts; cap the visible lean.
            if (normal.z() < params.mMinSupportZ)
                return {};
            const float angle = std::acos(std::clamp(normal.z(), -1.f, 1.f));
            osg::Vec3f axis = osg::Vec3f(0.f, 0.f, 1.f) ^ normal;
            if (axis.normalize() < 0.0001f)
                return {};
            return osg::Quat(std::min(angle, params.mMaxLean), axis);
        }

        osg::Quat update(const osg::Vec3f& normal, const osg::Quat& yaw, float duration, const Params& params = {})
        {
            if (std::isfinite(duration) && duration > 0.f)
            {
                const double blend = -std::expm1(-params.mResponsiveness * std::min(duration, 0.1f));
                osg::Quat next;
                next.slerp(blend, mWorldTilt, target(normal, params));
                mWorldTilt = next;
            }
            osg::Quat localTilt;
            localTilt.makeRotate(osg::Vec3f(0.f, 0.f, 1.f), yaw.inverse() * worldUp());
            return localTilt;
        }

        // The collision shape stays upright, so on a plane with this normal it rests on its uphill
        // edge and the origin (the centre of the shape's base) hovers above the plane by this much.
        static float restingGap(const osg::Vec3f& halfExtents, Shape shape, const osg::Quat& yaw, osg::Vec3f up)
        {
            if (shape == Shape::RotatingBox)
                up = yaw.inverse() * up;
            if (up.z() < 0.1f)
                return 0.f;
            float gap;
            if (shape == Shape::Cylinder)
                gap = halfExtents.x() * std::sqrt(up.x() * up.x() + up.y() * up.y()) / up.z();
            else
                gap = (halfExtents.x() * std::abs(up.x()) + halfExtents.y() * std::abs(up.y())) / up.z();
            return std::isfinite(gap) ? gap : 0.f;
        }

        // How far to lower the model, from the smoothed pose, so the base meets the slope plane.
        // Used off terrain, where the actual gap under the origin is not known.
        float sinkOffset(const osg::Vec3f& halfExtents, Shape shape, const osg::Quat& yaw, const Params& params) const
        {
            if (params.mSink <= 0.f)
                return 0.f;
            return params.mSink * restingGap(halfExtents, shape, yaw, worldUp());
        }

        // Plane through terrain heights sampled at the footprint's front, back, right and left
        // edges. Unlike the physics contact normal, which follows whichever terrain triangle the
        // hull edge touches this frame, it varies continuously as the actor walks.
        static osg::Vec3f footprintNormal(const osg::Vec3f& forward, const osg::Vec3f& right, float halfLength,
            float halfWidth, float front, float back, float rightZ, float leftZ)
        {
            const osg::Vec3f along = forward * (2.f * halfLength) + osg::Vec3f(0.f, 0.f, front - back);
            const osg::Vec3f across = right * (2.f * halfWidth) + osg::Vec3f(0.f, 0.f, rightZ - leftZ);
            osg::Vec3f normal = across ^ along;
            if (normal.z() < 0.f)
                normal = -normal;
            if (!std::isfinite(normal.x()) || !std::isfinite(normal.y()) || !std::isfinite(normal.z())
                || normal.normalize() < 0.0001f)
                return osg::Vec3f(0.f, 0.f, 1.f);
            return normal;
        }

        // Tilt that turns world up toward a surface normal for an item, or identity when the item
        // should stay upright.
        static osg::Quat itemTilt(const osg::Vec3f& normal, const ItemSlopeParams& params)
        {
            if (!params.mEnabled)
                return {};
            Params pose;
            pose.mMaxLean = params.mMaxTilt;
            pose.mMinSupportZ = params.mMinSupportZ;
            return target(normal, pose);
        }

        // Rotates an object about the point its base rests on, so that point stays in contact.
        // OSG composition: the result applies the object's rotation, then the tilt.
        static std::pair<osg::Vec3f, osg::Quat> tiltAbout(
            const osg::Vec3f& origin, const osg::Quat& rotation, const osg::Vec3f& pivot, const osg::Quat& tilt)
        {
            return { pivot + tilt * (origin - pivot), rotation * tilt };
        }

        osg::Vec3f worldUp() const { return mWorldTilt * osg::Vec3f(0.f, 0.f, 1.f); }

        void reset() { mWorldTilt = osg::Quat(); }

    private:
        osg::Quat mWorldTilt;
    };
}

#endif
