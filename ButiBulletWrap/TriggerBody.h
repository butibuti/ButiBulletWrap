#ifndef BUTIBULLET_TRIGGERBODY_H
#define BUTIBULLET_TRIGGERBODY_H
#include "Common.h"
#include "PhysicsObject.h"
#include "CollisionShape.h"

namespace ButiBullet {

class TriggerBody : public PhysicsObject,public ITriggerBody
{
public:
    BUTIBULLET_API static ButiEngine::Value_ptr<TriggerBody> Create(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);

    BUTIBULLET_API void AddCollisionShape(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);


    BUTIBULLET_API void SetTransform(const ButiEngine::Matrix4x4& arg_transform)override;
    BUTIBULLET_API const ButiEngine::Matrix4x4& GetTransform() const override{ return transform; }

    BUTIBULLET_API void SetCollisionGroup(const std::uint32_t arg_value) override;
    BUTIBULLET_API void SetCollisionGroupMask(const std::uint32_t arg_value) override;

    BUTIBULLET_API void OnDispose(const bool arg_explicitDisposing) override;
    BUTIBULLET_API void OnPrepareStepSimulation() override;
    BUTIBULLET_API void OnAfterStepSimulation() override;

    BUTIBULLET_API TriggerBody();
    BUTIBULLET_API void Initialize();
    BUTIBULLET_API void Initialize(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);

    inline const ButiEngine::Vector3& GetPosition()const override { return transform.GetPosition(); }
    inline void SetPosition(const ButiEngine::Vector3& arg_pos) override { transform.SetPosition(arg_pos); Activate(); }
    std::uint32_t GetCollisionGroup()const override { return group; }
    std::uint32_t GetCollisionGroupMask()const override { return groupMask; }
    void SetModifiedAll() { dirtyFlags = DirtyFlags_All; }
private:
    void RemoveFromBtWorld() override;
    void Activate();

    class LocalGhostObject;

    enum DirtyFlags
    {
        DirtyFlags_None = 0,

        DirtyFlags_InitialUpdate = 1 << 0,
        DirtyFlags_Shapes = 1 << 1,
        DirtyFlags_Group = 1 << 2,
        DirtyFlags_Transform = 1 << 3,

        DirtyFlags_All = 0xFFFF,
    };

    void CreateBtObject();
    void DeleteBtObject();
    void ReaddToWorld();

    std::uint32_t dirtyFlags = DirtyFlags_All;
    std::uint32_t group = 0x00000001;
    std::uint32_t groupMask = 0x0000FFFF;
    ButiEngine::Matrix4x4 transform;

    LocalGhostObject* p_btGhostObject = nullptr;
    bool btWorldAdded = false;

    PhysicsDetail::BtShapeManager shapeManager;
};

} 

#endif // !BUTIBULLET_TRIGGERBODY_H
