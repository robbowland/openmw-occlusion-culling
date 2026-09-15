#ifndef OPENMW_MWPHYSICS_PROJECTILE_H
#define OPENMW_MWPHYSICS_PROJECTILE_H

#include <atomic>
#include <memory>
#include <mutex>

#include <LinearMath/btVector3.h>
#include <components/misc/waterprojectile.hpp>

#include "ptrholder.hpp"

class btCollisionObject;
class btCollisionShape;
class btConvexShape;

namespace osg
{
    class Vec3f;
}

namespace MWPhysics
{
    class PhysicsTaskScheduler;
    class PhysicsSystem;

    class Projectile final : public PtrHolder
    {
    public:
        Projectile(const MWWorld::Ptr& caster, const osg::Vec3f& position, float radius,
            PhysicsTaskScheduler* scheduler, PhysicsSystem* physicssystem);
        ~Projectile() override;

        btConvexShape* getConvexShape() const { return mConvexShape; }

        void updateCollisionObjectPosition();

        bool isActive() const { return mActive.load(std::memory_order_acquire); }

        MWWorld::Ptr getTarget() const;

        MWWorld::Ptr getCaster() const;
        void setCaster(const MWWorld::Ptr& caster);
        const btCollisionObject* getCasterCollisionObject() const { return mCasterColObj; }

        void setHitWater() { mHitWater = true; }

        bool getHitWater() const { return mHitWater; }

        void hit(const btCollisionObject* target, btVector3 pos, btVector3 normal);

        void setValidTargets(const std::vector<MWWorld::Ptr>& targets);
        bool isValidTarget(const btCollisionObject* target) const;

        btVector3 getHitPosition() const { return mHitPosition; }
        btVector3 getHitNormal() const { return mHitNormal; }

        void enableWaterPenetration() { mWaterPenetration = true; }
        bool waterPenetrationEnabled() const { return mWaterPenetration; }
        std::unique_lock<std::mutex> lockWaterState() const { return std::unique_lock<std::mutex>(mWaterMutex); }
        Misc::WaterProjectile::State waterStateSnapshot() const
        {
            const auto lock = lockWaterState();
            return mWaterState;
        }
        void restoreWaterState(const Misc::WaterProjectile::State& state)
        {
            const auto lock = lockWaterState();
            mWaterState = state;
            mWaterRipplePending = false; // Do not replay an already emitted entry splash.
        }
        // Physics worker owns lockWaterState() while mutating this state.
        Misc::WaterProjectile::State& waterState() { return mWaterState; }
        void enterWater(btVector3 position)
        {
            mWaterState.entered = true;
            mWaterState.surface = position.z();
            mWaterEntryPosition = position;
            mWaterRipplePending = true;
        }
        bool takeWaterRipple(btVector3& position)
        {
            const auto lock = lockWaterState();
            if (!mWaterRipplePending) return false;
            position = mWaterEntryPosition;
            mWaterRipplePending = false;
            return true;
        }

    private:
        std::unique_ptr<btCollisionShape> mShape;
        btConvexShape* mConvexShape;

        bool mHitWater;
        bool mWaterPenetration = false;
        bool mWaterRipplePending = false;
        btVector3 mWaterEntryPosition;
        Misc::WaterProjectile::State mWaterState;
        mutable std::mutex mWaterMutex;
        std::atomic<bool> mActive;
        MWWorld::Ptr mCaster;
        const btCollisionObject* mCasterColObj;
        const btCollisionObject* mHitTarget;
        btVector3 mHitPosition;
        btVector3 mHitNormal;

        std::vector<const btCollisionObject*> mValidTargets;

        mutable std::mutex mMutex;

        PhysicsSystem* mPhysics;
        PhysicsTaskScheduler* mTaskScheduler;

        Projectile(const Projectile&);
        Projectile& operator=(const Projectile&);
    };

}

#endif
