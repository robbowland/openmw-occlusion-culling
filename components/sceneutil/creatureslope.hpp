#ifndef OPENMW_COMPONENTS_SCENEUTIL_CREATURESLOPE_H
#define OPENMW_COMPONENTS_SCENEUTIL_CREATURESLOPE_H

#include <osg/Quat>
#include <osg/Vec3f>

#include <algorithm>
#include <cmath>

namespace SceneUtil
{
    // Visual-only pose. Smooth in world space so turning does not drag the slope around.
    class CreatureSlope
    {
    public:
        static osg::Quat target(osg::Vec3f normal)
        {
            if (!std::isfinite(normal.x()) || !std::isfinite(normal.y()) || !std::isfinite(normal.z())
                || normal.length2() < 0.0001f)
                return {};
            normal.normalize();
            // Reject walls and extreme edge contacts; cap visible lean at 25 degrees.
            if (normal.z() < 0.70710678f)
                return {};
            const float angle = std::acos(std::clamp(normal.z(), -1.f, 1.f));
            osg::Vec3f axis = osg::Vec3f(0.f, 0.f, 1.f) ^ normal;
            if (axis.normalize() < 0.0001f)
                return {};
            return osg::Quat(std::min(angle, 0.43633231f), axis);
        }

        osg::Quat update(const osg::Vec3f& normal, const osg::Quat& yaw, float duration)
        {
            if (std::isfinite(duration) && duration > 0.f)
            {
                const double blend = -std::expm1(-10.0 * std::min(duration, 0.1f));
                osg::Quat next;
                next.slerp(blend, mWorldTilt, target(normal));
                mWorldTilt = next;
            }
            osg::Quat localTilt;
            localTilt.makeRotate(osg::Vec3f(0.f, 0.f, 1.f),
                yaw.inverse() * (mWorldTilt * osg::Vec3f(0.f, 0.f, 1.f)));
            return localTilt;
        }

        void reset() { mWorldTilt = osg::Quat(); }

    private:
        osg::Quat mWorldTilt;
    };
}

#endif
