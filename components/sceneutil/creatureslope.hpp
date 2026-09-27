#ifndef OPENMW_COMPONENTS_SCENEUTIL_CREATURESLOPE_H
#define OPENMW_COMPONENTS_SCENEUTIL_CREATURESLOPE_H

#include <osg/Quat>
#include <osg/Vec3f>

#include <algorithm>
#include <cmath>

namespace SceneUtil
{
    // Per-actor tuning, exposed to Lua through openmw.animation. Defaults reproduce the
    // original fixed behaviour: 25 degree lean cap, 45 degree support limit, rate 10, no sink.
    // Declared outside CreatureSlope so it can be a complete type in default arguments.
    struct CreatureSlopeParams
    {
        bool mEnabled = true;
        float mMaxLean = 0.43633231f; // radians
        float mMinSupportZ = 0.70710678f; // cosine of the steepest accepted support normal
        float mResponsiveness = 10.f; // exponential approach rate, per second
        float mSink = 0.f; // fraction of the collision-shape slope gap to lower the model by
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

        // The collision shape stays upright, so on a slope it rests on its uphill edge and the
        // model origin (the centre of the shape's base) hovers above the ground. Returns how far
        // to lower the model, from the smoothed pose, so the base meets the slope plane.
        float sinkOffset(const osg::Vec3f& halfExtents, Shape shape, const osg::Quat& yaw, const Params& params) const
        {
            if (params.mSink <= 0.f)
                return 0.f;
            osg::Vec3f up = worldUp();
            if (shape == Shape::RotatingBox)
                up = yaw.inverse() * up;
            if (up.z() < 0.1f)
                return 0.f;
            float gap;
            if (shape == Shape::Cylinder)
                gap = halfExtents.x() * std::sqrt(up.x() * up.x() + up.y() * up.y()) / up.z();
            else
                gap = (halfExtents.x() * std::abs(up.x()) + halfExtents.y() * std::abs(up.y())) / up.z();
            if (!std::isfinite(gap))
                return 0.f;
            return params.mSink * gap;
        }

        osg::Vec3f worldUp() const { return mWorldTilt * osg::Vec3f(0.f, 0.f, 1.f); }

        void reset() { mWorldTilt = osg::Quat(); }

    private:
        osg::Quat mWorldTilt;
    };
}

#endif
