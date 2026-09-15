#ifndef GAME_RENDER_CREATUREANIMATION_H
#define GAME_RENDER_CREATUREANIMATION_H

#include "../mwworld/inventorystore.hpp"
#include "actoranimation.hpp"
#include "weaponanimation.hpp"

#include <components/sceneutil/creatureslope.hpp>
#include <osg/MatrixTransform>

namespace MWWorld
{
    class Ptr;
}

namespace MWRender
{
    class CreatureAnimation : public ActorAnimation
    {
    public:
        CreatureAnimation(
            const MWWorld::Ptr& ptr, const std::string& model, Resource::ResourceSystem* resourceSystem, bool animated);
        ~CreatureAnimation() override;
        osg::Vec3f runAnimation(float duration) override;
        void removeFromScene() override;

    private:
        osg::ref_ptr<osg::Group> mSlopeParent;
        osg::ref_ptr<osg::MatrixTransform> mSlopeTransform;
        SceneUtil::CreatureSlope mSlope;
        osg::Vec3f mPreviousSlopePosition;
        bool mSlopeDebug = false;
        float mSlopeLogTimer = 0.f;
    };

    // For creatures with weapons and shields
    // Animation is already virtual anyway, so might as well make a separate class.
    // Most creatures don't need weapons/shields, so this will save some memory.
    class CreatureWeaponAnimation : public ActorAnimation,
                                    public WeaponAnimation,
                                    public MWWorld::InventoryStoreListener
    {
    public:
        CreatureWeaponAnimation(
            const MWWorld::Ptr& ptr, const std::string& model, Resource::ResourceSystem* resourceSystem, bool animated);
        virtual ~CreatureWeaponAnimation() {}

        void equipmentChanged() override { updateParts(); }

        bool getWeaponsShown() const override { return mShowWeapons; }
        void showWeapons(bool showWeapon) override;

        bool getCarriedLeftShown() const override { return mShowCarriedLeft; }
        void showCarriedLeft(bool show) override;

        void updateParts();

        void updatePart(PartHolderPtr& scene, int slot);

        void attachArrow() override;
        void detachArrow() override;
        void releaseArrow(float attackStrength, float attackWindUp) override;
        // WeaponAnimation
        osg::Group* getArrowBone() override;
        osg::Node* getWeaponNode() override;
        Resource::ResourceSystem* getResourceSystem() override;
        void showWeapon(bool show) override { showWeapons(show); }
        void setWeaponGroup(const std::string& group, bool relativeDuration) override
        {
            mWeaponAnimationTime->setGroup(group, relativeDuration);
        }

        void addControllers() override;

        osg::Vec3f runAnimation(float duration) override;

        /// A relative factor (0-1) that decides if and how much the skeleton should be pitched
        /// to indicate the facing orientation of the character.
        void setPitchFactor(float factor) override { mPitchFactor = factor; }

    protected:
        bool isArrowAttached() const override;

    private:
        PartHolderPtr mWeapon;
        PartHolderPtr mShield;
        bool mShowWeapons;
        bool mShowCarriedLeft;

        std::shared_ptr<WeaponAnimationTime> mWeaponAnimationTime;
    };
}

#endif
