#ifndef OPENMW_MISC_WATERPROJECTILE_H
#define OPENMW_MISC_WATERPROJECTILE_H

#include <algorithm>
#include <cmath>

namespace Misc::WaterProjectile
{
    // Experimental gameplay approximation, not a fluid simulation.
    inline constexpr float MaxDepth = 64.f;
    inline constexpr float MaxTravel = 128.f;
    inline constexpr float EntrySpeed = 0.5f;
    inline constexpr float Drag = 3.f;
    inline constexpr float MaxTime = 1.5f;

    struct State
    {
        bool entered = false;
        float surface = 0.f;
        float travel = 0.f;
        float age = 0.f;
    };

    inline float speedFactor(float age) { return EntrySpeed * std::exp(-Drag * age); }
    // Low ballistic arc for the air segment. Stable form avoids cancellation
    // at shallow angles. Return the original elevation if no solution exists.
    inline float ballisticAimZ(float horizontal, float elevation, float speed, float gravity)
    {
        if (horizontal <= 0.f || speed <= 0.f || gravity <= 0.f) return elevation;
        const double r=horizontal, z=elevation, v2=double(speed)*speed, g=gravity;
        const double term=g*r*r+2*z*v2;
        const double discriminant=v2*v2-g*term;
        if (discriminant < 0) return elevation;
        return static_cast<float>(term/(v2+std::sqrt(discriminant)));
    }
    inline float stepDistance(float speed, float age, float dt)
    {
        return speed * speedFactor(age) * -std::expm1(-Drag * dt) / Drag;
    }
    inline float allowedDistance(const State& state, float z, float directionZ, float proposed)
    {
        float allowed = std::min(proposed, std::max(0.f, MaxTravel - state.travel));
        if (directionZ < 0.f)
            allowed = std::min(allowed, std::max(0.f, z - (state.surface - MaxDepth)) / -directionZ);
        return std::max(0.f, allowed);
    }
}
#endif
