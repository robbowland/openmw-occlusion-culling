#include "actoranimation.hpp"
#include <utility>

#include <osg/Group>
#include <osg/Node>
#include <osg/Vec4f>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadbody.hpp>
#include <components/esm3/loadcell.hpp>
#include <components/esm3/loadcrea.hpp>
#include <components/esm3/loadligh.hpp>

#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>

#include <components/sceneutil/attach.hpp>
#include <components/sceneutil/lightcommon.hpp>
#include <components/sceneutil/lightmanager.hpp>
#include <components/sceneutil/lightutil.hpp>
#include <components/sceneutil/visitor.hpp>

#include <components/misc/resourcehelpers.hpp>

#include <components/settings/values.hpp>

#include <components/vfs/manager.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/drawstate.hpp"
#include "../mwmechanics/weapontype.hpp"
#include "../mwworld/cellstore.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/ptr.hpp"

#include "actorutil.hpp"
#include "vismask.hpp"

namespace MWRender
{

    ActorAnimation::ActorAnimation(
        const MWWorld::Ptr& ptr, osg::ref_ptr<osg::Group> parentNode, Resource::ResourceSystem* resourceSystem)
        : Animation(ptr, std::move(parentNode), resourceSystem)
    {
        MWWorld::ContainerStore& store = mPtr.getClass().getContainerStore(mPtr);

        for (MWWorld::ConstContainerStoreIterator iter = store.cbegin(MWWorld::ContainerStore::Type_Light);
             iter != store.cend(); ++iter)
        {
            const ESM::Light* light = iter->get<ESM::Light>()->mBase;
            if (!(light->mData.mFlags & ESM::Light::Carry))
            {
                addHiddenItemLight(*iter, light);
            }
        }

        // Make sure we cleaned object from effects, just in cast if we re-use node
        removeEffects();

        // The player is never posed: first person, the inventory preview and the camera rely on
        // an upright model.
        if (Settings::game().mCreatureSlopeAlignment && mPtr != MWMechanics::getPlayer())
        {
            if (mPtr.getType() == ESM::Creature::sRecordId)
            {
                const int flags = mPtr.get<ESM::Creature>()->mBase->mFlags;
                mSlopeAliveEligible = SceneUtil::CreatureSlope::eligible(
                    flags & ESM::Creature::Walks, flags & ESM::Creature::Bipedal, flags & ESM::Creature::Flies);
            }
            mSlopeParent = mInsert;
            mSlopeTransform = new osg::MatrixTransform;
            mSlopeTransform->setDataVariance(osg::Object::DYNAMIC);
            mSlopeParent->addChild(mSlopeTransform);
            mInsert = mSlopeTransform;
            mPreviousSlopePosition = mPtr.getRefData().getPosition().asVec3();
            mSlopeDebug = Settings::game().mCreatureSlopeDebug;
        }
    }

    ActorAnimation::~ActorAnimation()
    {
        removeFromSceneImpl();
    }

    PartHolderPtr ActorAnimation::attachMesh(
        VFS::Path::NormalizedView model, std::string_view bonename, const osg::Vec4f* glowColor)
    {
        osg::Group* parent = getBoneByName(bonename);
        if (!parent)
            return nullptr;

        osg::ref_ptr<osg::Node> instance = mResourceSystem->getSceneManager()->getInstance(model, parent);

        const NodeMap& nodeMap = getNodeMap();
        NodeMap::const_iterator found = nodeMap.find(bonename);
        if (found == nodeMap.end())
            return {};

        if (glowColor != nullptr)
            mGlowUpdater = SceneUtil::addEnchantedGlow(instance, mResourceSystem, *glowColor);

        return std::make_unique<PartHolder>(instance);
    }

    osg::ref_ptr<osg::Node> ActorAnimation::attach(
        VFS::Path::NormalizedView model, std::string_view bonename, std::string_view bonefilter, bool isLight)
    {
        osg::ref_ptr<const osg::Node> templateNode = mResourceSystem->getSceneManager()->getTemplate(model);

        const NodeMap& nodeMap = getNodeMap();
        auto found = nodeMap.find(bonename);
        if (found == nodeMap.end())
            throw std::runtime_error("Can't find attachment node " + std::string{ bonename });
        if (isLight)
        {
            osg::Quat rotation(osg::DegreesToRadians(-90.f), osg::Vec3f(1, 0, 0));
            return SceneUtil::attach(
                templateNode, mObjectRoot, bonefilter, found->second, mResourceSystem->getSceneManager(), &rotation);
        }
        return SceneUtil::attach(
            std::move(templateNode), mObjectRoot, bonefilter, found->second, mResourceSystem->getSceneManager());
    }

    VFS::Path::Normalized ActorAnimation::getShieldMesh(const MWWorld::ConstPtr& shield, bool female) const
    {
        const ESM::Armor* armor = shield.get<ESM::Armor>()->mBase;
        const std::vector<ESM::PartReference>& bodyparts = armor->mParts.mParts;
        // Try to recover the body part model, use ground model as a fallback otherwise.
        if (!bodyparts.empty())
        {
            const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
            const MWWorld::Store<ESM::BodyPart>& partStore = store.get<ESM::BodyPart>();
            for (const auto& part : bodyparts)
            {
                if (part.mPart != ESM::PRT_Shield)
                    continue;

                const ESM::RefId* bodypartName = nullptr;
                if (female && !part.mFemale.empty())
                    bodypartName = &part.mFemale;
                else if (!part.mMale.empty())
                    bodypartName = &part.mMale;

                if (bodypartName && !bodypartName->empty())
                {
                    const ESM::BodyPart* bodypart = partStore.search(*bodypartName);
                    if (bodypart == nullptr || bodypart->mData.mType != ESM::BodyPart::MT_Armor)
                        return VFS::Path::Normalized();
                    if (!bodypart->mModel.getNormalized().empty())
                        return Misc::ResourceHelpers::correctMeshPath(bodypart->mModel.getNormalized());
                }
            }
        }
        return shield.getClass().getCorrectedModel(shield);
    }

    std::string ActorAnimation::getSheathedShieldMesh(const MWWorld::ConstPtr& shield) const
    {
        std::string mesh = getShieldMesh(shield, false);

        if (mesh.empty())
            return mesh;

        const VFS::Path::Normalized holsteredName(addSuffixBeforeExtension(mesh, "_sh"));
        if (mResourceSystem->getVFS()->exists(holsteredName))
        {
            osg::ref_ptr<osg::Node> shieldTemplate = mResourceSystem->getSceneManager()->getInstance(holsteredName);
            SceneUtil::FindByNameVisitor findVisitor("Bip01 Sheath");
            shieldTemplate->accept(findVisitor);
            osg::ref_ptr<osg::Node> sheathNode = findVisitor.mFoundNode;
            if (!sheathNode)
                return std::string();
        }

        return mesh;
    }

    bool ActorAnimation::updateCarriedLeftVisible(ESM::RefId weaptype) const
    {
        if (Settings::game().mShieldSheathing && mObjectRoot)
        {
            const MWWorld::Class& cls = mPtr.getClass();
            MWMechanics::CreatureStats& stats = cls.getCreatureStats(mPtr);
            if (cls.hasInventoryStore(mPtr) && stats.getDrawState() == MWMechanics::DrawState::Nothing)
            {
                SceneUtil::FindByNameVisitor findVisitor("Bip01 AttachShield");
                mObjectRoot->accept(findVisitor);
                if (findVisitor.mFoundNode)
                {
                    const MWWorld::InventoryStore& inv = cls.getInventoryStore(mPtr);
                    const MWWorld::ConstContainerStoreIterator shield
                        = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                    if (shield != inv.end() && shield->getType() == ESM::Armor::sRecordId
                        && !getSheathedShieldMesh(*shield).empty())
                        return false;
                }
            }
        }

        return !(MWMechanics::getWeaponType(weaptype)->mFlags & ESM::WeaponType::TwoHanded);
    }

    void ActorAnimation::updateHolsteredShield(bool showCarriedLeft)
    {
        if (!Settings::game().mShieldSheathing)
            return;

        if (!mPtr.getClass().hasInventoryStore(mPtr))
            return;

        mHolsteredShield.reset();

        if (showCarriedLeft)
            return;

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (shield == inv.end() || shield->getType() != ESM::Armor::sRecordId)
            return;

        // Can not show holdstered shields with two-handed weapons at all
        const MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end())
            return;

        auto type = weapon->getType();
        if (type == ESM::Weapon::sRecordId)
        {
            const MWWorld::LiveCellRef<ESM::Weapon>* ref = weapon->get<ESM::Weapon>();
            const ESM::RefId weaponType = ref->mBase->mData.mType;
            if (MWMechanics::getWeaponType(weaponType)->mFlags & ESM::WeaponType::TwoHanded)
                return;
        }

        const VFS::Path::Normalized mesh = getSheathedShieldMesh(*shield);
        if (mesh.empty())
            return;

        constexpr std::string_view boneName = "Bip01 AttachShield";
        const bool isEnchanted = !shield->getClass().getEnchantment(*shield).empty();
        const osg::Vec4f glowColor = isEnchanted ? shield->getClass().getEnchantmentColor(*shield) : osg::Vec4f();
        const VFS::Path::Normalized holsteredName = addSuffixBeforeExtension(mesh, "_sh");

        // If we have no dedicated sheath model, use basic shield model as fallback.
        if (!mResourceSystem->getVFS()->exists(holsteredName))
            mHolsteredShield = attachMesh(mesh, boneName, isEnchanted ? &glowColor : nullptr);
        else
            mHolsteredShield = attachMesh(holsteredName, boneName, isEnchanted ? &glowColor : nullptr);

        if (!mHolsteredShield)
            return;

        SceneUtil::FindByNameVisitor findVisitor("Bip01 Sheath");
        mHolsteredShield->getNode()->accept(findVisitor);
        osg::Group* shieldNode = findVisitor.mFoundNode;

        // If mesh author declared an empty sheath node, use transformation from this node, but use the common shield
        // mesh. This approach allows to tweak shield position without need to store the whole shield mesh in the _sh
        // file.
        if (shieldNode && !shieldNode->getNumChildren())
        {
            osg::ref_ptr<osg::Node> fallbackNode = mResourceSystem->getSceneManager()->getInstance(mesh, shieldNode);
            if (isEnchanted)
                SceneUtil::addEnchantedGlow(shieldNode, mResourceSystem, glowColor);
        }
    }

    bool ActorAnimation::useShieldAnimations() const
    {
        if (!Settings::game().mShieldSheathing)
            return false;

        const MWWorld::Class& cls = mPtr.getClass();
        if (!cls.hasInventoryStore(mPtr))
            return false;

        if (getTextKeyTime("shield: equip attach") < 0 || getTextKeyTime("shield: unequip detach") < 0)
            return false;

        const MWWorld::InventoryStore& inv = cls.getInventoryStore(mPtr);
        const MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        const MWWorld::ConstContainerStoreIterator shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (weapon != inv.end() && shield != inv.end() && shield->getType() == ESM::Armor::sRecordId
            && !getSheathedShieldMesh(*shield).empty())
        {
            auto type = weapon->getType();
            if (type == ESM::Weapon::sRecordId)
            {
                const MWWorld::LiveCellRef<ESM::Weapon>* ref = weapon->get<ESM::Weapon>();
                const ESM::RefId weaponType = ref->mBase->mData.mType;
                return !(MWMechanics::getWeaponType(weaponType)->mFlags & ESM::WeaponType::TwoHanded);
            }
            else if (type == ESM::Lockpick::sRecordId || type == ESM::Probe::sRecordId)
                return true;
        }

        return false;
    }

    osg::Group* ActorAnimation::getBoneByName(std::string_view boneName) const
    {
        if (!mObjectRoot)
            return nullptr;

        SceneUtil::FindByNameVisitor findVisitor(boneName);
        mObjectRoot->accept(findVisitor);

        return findVisitor.mFoundNode;
    }

    std::string_view ActorAnimation::getHolsteredWeaponBoneName(const MWWorld::ConstPtr& weapon)
    {
        if (weapon.isEmpty())
            return {};

        auto type = weapon.getClass().getType();
        if (type == ESM::Weapon::sRecordId)
        {
            const MWWorld::LiveCellRef<ESM::Weapon>* ref = weapon.get<ESM::Weapon>();
            const ESM::RefId weaponType = ref->mBase->mData.mType;
            return MWMechanics::getWeaponType(weaponType)->mSheathingBone;
        }

        return {};
    }

    void ActorAnimation::resetControllers(osg::Node* node)
    {
        if (node == nullptr)
            return;

        // This is used to avoid playing animations intended for equipped weapons on holstered weapons.
        SceneUtil::ForceControllerSourcesVisitor removeVisitor(std::make_shared<NullAnimationTime>());
        node->accept(removeVisitor);
    }

    void ActorAnimation::updateHolsteredWeapon(bool showHolsteredWeapons)
    {
        if (!Settings::game().mWeaponSheathing)
            return;

        if (!mPtr.getClass().hasInventoryStore(mPtr))
            return;

        mScabbard.reset();

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return;

        // Since throwing weapons stack themselves, do not show such weapon itself
        const ESM::RefId type = weapon->get<ESM::Weapon>()->mBase->mData.mType;
        auto weaponClass = MWMechanics::getWeaponType(type)->mWeaponClass;
        if (weaponClass == ESM::WeaponType::Thrown)
            showHolsteredWeapons = false;

        const VFS::Path::Normalized mesh = weapon->getClass().getCorrectedModel(*weapon);
        if (mesh.empty())
            return;

        const std::string_view boneName = getHolsteredWeaponBoneName(*weapon);
        if (boneName.empty())
            return;

        // If the scabbard is not found, use the weapon mesh as fallback.
        const VFS::Path::Normalized scabbardName = addSuffixBeforeExtension(mesh, "_sh");
        const bool isEnchanted = !weapon->getClass().getEnchantment(*weapon).empty();
        if (!mResourceSystem->getVFS()->exists(scabbardName))
        {
            if (showHolsteredWeapons)
            {
                const osg::Vec4f glowColor
                    = isEnchanted ? weapon->getClass().getEnchantmentColor(*weapon) : osg::Vec4f();
                mScabbard = attachMesh(mesh, boneName, isEnchanted ? &glowColor : nullptr);
                if (mScabbard && weaponClass == ESM::WeaponType::Ranged)
                    resetControllers(mScabbard->getNode());
            }

            return;
        }

        mScabbard = attachMesh(scabbardName, boneName);

        if (!mScabbard || !mScabbard->getNode())
            return;

        if (weaponClass == ESM::WeaponType::Ranged)
            resetControllers(mScabbard->getNode());

        SceneUtil::FindByNameVisitor findVisitor("Bip01 Weapon");
        mScabbard->getNode()->accept(findVisitor);
        osg::Group* weaponNode = findVisitor.mFoundNode;

        if (!weaponNode)
            return;

        // When we draw weapon, hide the Weapon node from sheath model.
        // Otherwise add the enchanted glow to it.
        if (!showHolsteredWeapons)
        {
            weaponNode->setNodeMask(0);
        }
        else
        {
            // If mesh author declared empty weapon node, use transformation from this node, but use the common weapon
            // mesh. This approach allows to tweak weapon position without need to store the whole weapon mesh in the
            // _sh file.
            if (!weaponNode->getNumChildren())
            {
                osg::ref_ptr<osg::Node> fallbackNode
                    = mResourceSystem->getSceneManager()->getInstance(mesh, weaponNode);

                if (weaponClass == ESM::WeaponType::Ranged)
                    resetControllers(fallbackNode);
            }

            if (isEnchanted)
            {
                osg::Vec4f glowColor = weapon->getClass().getEnchantmentColor(*weapon);
                mGlowUpdater = SceneUtil::addEnchantedGlow(weaponNode, mResourceSystem, glowColor);
            }
        }
    }

    void ActorAnimation::updateQuiver()
    {
        if (!Settings::game().mWeaponSheathing)
            return;

        if (!mPtr.getClass().hasInventoryStore(mPtr))
            return;

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return;

        const VFS::Path::NormalizedView mesh = weapon->getClass().getModel(*weapon);
        std::string_view boneName = getHolsteredWeaponBoneName(*weapon);
        if (mesh.empty() || boneName.empty())
            return;

        osg::Group* ammoNode = getBoneByName("Bip01 Ammo");
        if (!ammoNode)
            return;

        // Special case for throwing weapons - they do not use ammo, but they stack themselves
        bool suitableAmmo = false;
        MWWorld::ConstContainerStoreIterator ammo = weapon;
        unsigned int ammoCount = 0;
        const ESM::RefId type = weapon->get<ESM::Weapon>()->mBase->mData.mType;
        const auto& weaponType = MWMechanics::getWeaponType(type);
        if (weaponType->mWeaponClass == ESM::WeaponType::Thrown)
        {
            ammoCount = ammo->getCellRef().getCount();
            osg::Group* throwingWeaponNode = getBoneByName(weaponType->mAttachBone);
            if (throwingWeaponNode && throwingWeaponNode->getNumChildren())
                ammoCount--;

            suitableAmmo = true;
        }
        else
        {
            ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
            if (ammo == inv.end())
                return;

            ammoCount = ammo->getCellRef().getCount();
            bool arrowAttached = isArrowAttached();
            if (arrowAttached)
                ammoCount--;

            suitableAmmo = ammo->get<ESM::Weapon>()->mBase->mData.mType == weaponType->mAmmoType;
        }

        if (!suitableAmmo)
            return;

        // We should not show more ammo than equipped and more than quiver mesh has
        ammoCount = std::min(ammoCount, ammoNode->getNumChildren());

        // Remove existing ammo nodes
        for (unsigned int i = 0; i < ammoNode->getNumChildren(); ++i)
        {
            osg::ref_ptr<osg::Group> arrowNode = ammoNode->getChild(i)->asGroup();
            if (!arrowNode->getNumChildren())
                continue;

            osg::ref_ptr<osg::Node> arrowChildNode = arrowNode->getChild(0);
            arrowNode->removeChild(arrowChildNode);
        }

        // Add new ones
        osg::Vec4f glowColor = ammo->getClass().getEnchantmentColor(*ammo);
        const VFS::Path::Normalized model(ammo->getClass().getCorrectedModel(*ammo));
        for (unsigned int i = 0; i < ammoCount; ++i)
        {
            osg::ref_ptr<osg::Group> arrowNode = ammoNode->getChild(i)->asGroup();
            osg::ref_ptr<osg::Node> arrow = mResourceSystem->getSceneManager()->getInstance(model, arrowNode);
            if (!ammo->getClass().getEnchantment(*ammo).empty())
                mGlowUpdater = SceneUtil::addEnchantedGlow(std::move(arrow), mResourceSystem, glowColor);
        }
    }

    void ActorAnimation::itemAdded(const MWWorld::ConstPtr& item, int /*count*/)
    {
        if (item.getType() == ESM::Light::sRecordId)
        {
            const ESM::Light* light = item.get<ESM::Light>()->mBase;
            if (!(light->mData.mFlags & ESM::Light::Carry))
            {
                addHiddenItemLight(item, light);
            }
        }

        if (!mPtr.getClass().hasInventoryStore(mPtr))
            return;

        // If the count of equipped ammo or throwing weapon was changed, we should update quiver
        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return;

        MWWorld::ConstContainerStoreIterator ammo = inv.end();
        const ESM::RefId type = weapon->get<ESM::Weapon>()->mBase->mData.mType;
        if (MWMechanics::getWeaponType(type)->mWeaponClass == ESM::WeaponType::Thrown)
            ammo = weapon;
        else
            ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);

        if (ammo != inv.end() && item.getCellRef().getRefId() == ammo->getCellRef().getRefId())
            updateQuiver();
    }

    void ActorAnimation::itemRemoved(const MWWorld::ConstPtr& item, int /*count*/)
    {
        if (item.getType() == ESM::Light::sRecordId)
        {
            ItemLightMap::iterator iter = mItemLights.find(item);
            if (iter != mItemLights.end())
            {
                if (!item.getCellRef().getCount())
                {
                    removeHiddenItemLight(item);
                }
            }
        }

        if (!mPtr.getClass().hasInventoryStore(mPtr))
            return;

        // If the count of equipped ammo or throwing weapon was changed, we should update quiver
        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return;

        MWWorld::ConstContainerStoreIterator ammo = inv.end();
        const ESM::RefId type = weapon->get<ESM::Weapon>()->mBase->mData.mType;
        if (MWMechanics::getWeaponType(type)->mWeaponClass == ESM::WeaponType::Thrown)
            ammo = weapon;
        else
            ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);

        if (ammo != inv.end() && item.getCellRef().getRefId() == ammo->getCellRef().getRefId())
            updateQuiver();
    }

    void ActorAnimation::addHiddenItemLight(const MWWorld::ConstPtr& item, const ESM::Light* esmLight)
    {
        if (mItemLights.find(item) != mItemLights.end())
            return;

        bool exterior = mPtr.isInCell() && mPtr.getCell()->getCell()->isExterior();

        osg::Vec4f ambient(1, 1, 1, 1);
        osg::ref_ptr<SceneUtil::LightSource> lightSource
            = SceneUtil::createLightSource(SceneUtil::LightCommon(*esmLight), Mask_Lighting, exterior, ambient);

        mInsert->addChild(lightSource);

        if (mLightListCallback && mPtr == MWMechanics::getPlayer())
            mLightListCallback->getIgnoredLightSources().insert(lightSource.get());

        mItemLights.insert(std::make_pair(item, lightSource));
    }

    void ActorAnimation::removeHiddenItemLight(const MWWorld::ConstPtr& item)
    {
        ItemLightMap::iterator iter = mItemLights.find(item);
        if (iter == mItemLights.end())
            return;

        if (mLightListCallback && mPtr == MWMechanics::getPlayer())
        {
            std::set<SceneUtil::LightSource*>::iterator ignoredIter
                = mLightListCallback->getIgnoredLightSources().find(iter->second.get());
            if (ignoredIter != mLightListCallback->getIgnoredLightSources().end())
                mLightListCallback->getIgnoredLightSources().erase(ignoredIter);
        }

        mInsert->removeChild(iter->second);
        mItemLights.erase(iter);
    }

    void ActorAnimation::removeFromScene()
    {
        removeFromSceneImpl();
        Animation::removeFromScene();
    }

    void ActorAnimation::removeFromSceneImpl()
    {
        for (const auto& [k, v] : mItemLights)
            mInsert->removeChild(v);
        if (mSlopeParent && mSlopeTransform)
            mSlopeParent->removeChild(mSlopeTransform);
    }

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

    void ActorAnimation::updateSlopePose(float duration)
    {
        if (!mSlopeTransform)
            return;
        const bool dead = mPtr.getClass().getCreatureStats(mPtr).isDead();
        if (!dead && !mSlopeAliveEligible)
        {
            // Living NPCs and bipeds stay upright; only reset once (after a resurrection).
            if (!mSlopeUpright)
            {
                mSlope.reset();
                mSlopeSink = mSlopeSinkResidual = 0.f;
                mSlopeOnTerrain = false;
                mSlopeNormal.set(0.f, 0.f, 1.f);
                mSlopeTransform->setMatrix(osg::Matrix::identity());
                mSlopeUpright = true;
            }
            return;
        }
        mSlopeUpright = false;

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

        SceneUtil::CreatureSlope::Params params = mSlopeParams;
        if (dead)
            params.mMaxLean = params.mCorpseMaxLean;

        osg::Vec3f normal(0.f, 0.f, 1.f);
        float sink = 0.f;
        bool onTerrain = false;
        if (params.mEnabled)
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
                    sink = params.mSink * std::max(0.f, gap - 1.f);
                }
            }
            if (!onTerrain && contact)
                normal = *contact;
        }
        const osg::Quat tilt = mSlope.update(normal, yaw, duration, params);
        mSlopeNormal = normal;
        if (!onTerrain && params.mSink > 0.f)
            sink = mSlope.sinkOffset(world->getHalfExtents(mPtr), slopeShape(), yaw, params);
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
                                 << renderedUp.z() << " sink=" << mSlopeSink << " terrain=" << onTerrain
                                 << " dead=" << dead;
            }
        }
    }

    std::optional<Animation::GroundAlignmentState> ActorAnimation::getGroundAlignment() const
    {
        if (!mSlopeTransform)
            return std::nullopt;
        return GroundAlignmentState{ mSlopeParams, mSlopeNormal, mSlope.worldUp(), mSlopeSink, mSlopeOnTerrain };
    }

    bool ActorAnimation::setGroundAlignment(const SceneUtil::CreatureSlope::Params& params)
    {
        if (!mSlopeTransform)
            return false;
        mSlopeParams = params;
        return true;
    }
}
