#ifndef OPENMW_ESM_PROJECTILESTATE_H
#define OPENMW_ESM_PROJECTILESTATE_H

#include <osg/Quat>
#include <osg/Vec3f>

#include "components/esm/quaternion.hpp"
#include "components/esm/fourcc.hpp"
#include "components/esm/refid.hpp"
#include "components/esm/vector3.hpp"

#include "refnum.hpp"

namespace ESM
{
    // Save-only record. Unpatched engines skip this whole transient projectile.
    inline constexpr auto WaterProjectileRecord = fourCC("WPRJ");
    class ESMReader;
    class ESMWriter;

    // format 0, savegames only

    struct BaseProjectileState
    {
        RefId mId;

        Vector3 mPosition;
        Quaternion mOrientation;

        RefNum mCaster;

        void load(ESMReader& esm);
        void save(ESMWriter& esm) const;
    };

    struct MagicBoltState : public BaseProjectileState
    {
        RefId mSpellId;
        float mSpeed;
        RefNum mItem;

        void load(ESMReader& esm);
        void save(ESMWriter& esm) const;
    };

    struct ProjectileState : public BaseProjectileState
    {
        RefId mBowId;
        Vector3 mVelocity;
        float mAttackStrength;
        float mAttackWindUp;

        // Optional save-only water budget: entered, surface, travel, age.
        // Absent in old saves and omitted for arrows that have not entered water.
        float mWater[4] = {};

        void load(ESMReader& esm);
        void save(ESMWriter& esm) const;
        void saveWater(ESMWriter& esm) const;
    };

}

#endif
