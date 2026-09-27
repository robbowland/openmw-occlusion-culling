#ifndef GAME_RENDER_ACTORANIMATION_H
#define GAME_RENDER_ACTORANIMATION_H

#include <map>

#include <osg/MatrixTransform>
#include <osg/ref_ptr>

#include <components/sceneutil/creatureslope.hpp>

#include <components/vfs/pathutil.hpp>

#include "../mwworld/containerstore.hpp"

#include "animation.hpp"

namespace osg
{
    class Node;
}

namespace MWWorld
{
    class ConstPtr;
}

namespace SceneUtil
{
    class LightSource;
    class LightListCallback;
}

namespace MWRender
{

    class ActorAnimation : public Animation, public MWWorld::ContainerStoreListener
    {
    public:
        ActorAnimation(
            const MWWorld::Ptr& ptr, osg::ref_ptr<osg::Group> parentNode, Resource::ResourceSystem* resourceSystem);
        virtual ~ActorAnimation();

        void itemAdded(const MWWorld::ConstPtr& item, int count) override;
        void itemRemoved(const MWWorld::ConstPtr& item, int count) override;
        virtual bool isArrowAttached() const { return false; }
        bool useShieldAnimations() const override;
        bool updateCarriedLeftVisible(ESM::RefId weaptype) const override;

        void removeFromScene() override;

        std::optional<GroundAlignmentState> getGroundAlignment() const override;
        bool setGroundAlignment(const SceneUtil::CreatureSlope::Params& params) override;

    protected:
        // Visual slope pose: live walking quadrupeds, and every actor's corpse. Call once per
        // frame after the animation has run; root motion is unaffected.
        void updateSlopePose(float duration);

        osg::Group* getBoneByName(std::string_view boneName) const;
        void updateHolsteredWeapon(bool showHolsteredWeapons);
        void updateHolsteredShield(bool showCarriedLeft);
        void updateQuiver();
        VFS::Path::Normalized getShieldMesh(const MWWorld::ConstPtr& shield, bool female) const;
        virtual std::string getSheathedShieldMesh(const MWWorld::ConstPtr& shield) const;
        std::string_view getHolsteredWeaponBoneName(const MWWorld::ConstPtr& weapon);

        PartHolderPtr attachMesh(
            VFS::Path::NormalizedView model, std::string_view bonename, const osg::Vec4f* glowColor = nullptr);

        osg::ref_ptr<osg::Node> attach(
            VFS::Path::NormalizedView model, std::string_view bonename, std::string_view bonefilter, bool isLight);

        PartHolderPtr mScabbard;
        PartHolderPtr mHolsteredShield;

    private:
        void addHiddenItemLight(const MWWorld::ConstPtr& item, const ESM::Light* esmLight);
        void removeHiddenItemLight(const MWWorld::ConstPtr& item);
        void resetControllers(osg::Node* node);
        void removeFromSceneImpl();

        typedef std::map<MWWorld::ConstPtr, osg::ref_ptr<SceneUtil::LightSource>> ItemLightMap;
        ItemLightMap mItemLights;

        osg::ref_ptr<osg::Group> mSlopeParent;
        osg::ref_ptr<osg::MatrixTransform> mSlopeTransform;
        SceneUtil::CreatureSlope mSlope;
        SceneUtil::CreatureSlope::Params mSlopeParams;
        osg::Vec3f mSlopeNormal{ 0.f, 0.f, 1.f };
        osg::Vec3f mPreviousSlopePosition;
        float mSlopeSink = 0.f;
        float mSlopeSinkResidual = 0.f;
        float mSlopeLogTimer = 0.f;
        bool mSlopeOnTerrain = false;
        bool mSlopeAliveEligible = false; // walking, non-bipedal, non-flying creature
        bool mSlopeUpright = true; // transform currently identity
        bool mSlopeDebug = false;
    };

}

#endif
