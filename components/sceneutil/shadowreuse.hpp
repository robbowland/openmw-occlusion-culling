#ifndef OPENMW_COMPONENTS_SCENEUTIL_SHADOWREUSE_H
#define OPENMW_COMPONENTS_SCENEUTIL_SHADOWREUSE_H

#include <algorithm>
#include <cmath>

namespace SceneUtil
{
    // FreeFPS temporal cadence is per CullVisitor, not per global frame: OSG may
    // alternate visitors. Never reuse a texture longer than maxAge after rendering
    // (default 40 ms). A fixed age cap silently disables reuse below ~25 fps, which
    // is where reuse matters most, so the cap is a setting.
    struct ShadowReuse
    {
        unsigned int mFramesSinceUpdate = 0;
        double mLastUpdate = 0.0;

        bool reuse(unsigned int interval, double now, double maxAge = 0.040)
        {
            const double age = now - mLastUpdate;
            if (interval <= 1 || mFramesSinceUpdate == 0 || mFramesSinceUpdate >= interval
                || !std::isfinite(age) || age < 0.0 || age > maxAge)
                return false;
            ++mFramesSinceUpdate;
            return true;
        }

        void recordUpdate(double now)
        {
            mFramesSinceUpdate = 1;
            mLastUpdate = now;
        }

        void invalidate() { mFramesSinceUpdate = 0; }
    };

    inline void expandAndSnapShadowRange(double& minimum, double& maximum, double resolution, double margin)
    {
        if (!std::isfinite(minimum) || !std::isfinite(maximum) || maximum <= minimum
            || !std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(margin))
            return;
        minimum = std::max(-1.0, minimum - margin);
        maximum = std::min(1.0, maximum + margin);
        const double range = maximum - minimum;
        if (range <= 0.0)
            return;
        const double texelSize = range / resolution;
        minimum = std::floor(minimum / texelSize) * texelSize;
        maximum = minimum + range;
    }
}

#endif
