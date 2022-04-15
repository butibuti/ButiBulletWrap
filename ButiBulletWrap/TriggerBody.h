#ifndef BUTIBULLET_TRIGGERBODY_H
#define BUTIBULLET_TRIGGERBODY_H
#include "Common.h"
#include "PhysicsObject.h"
#include "CollisionShape.h"

namespace ButiBullet {

class TriggerBody : public PhysicsObject
{
public:
    BUTIBULLET_API static ButiEngine::Value_ptr<TriggerBody> Create(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);

    BUTIBULLET_API void AddCollisionShape(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);


    BUTIBULLET_API void SetTransform(const ButiEngine::Matrix4x4& arg_transform);
    BUTIBULLET_API const ButiEngine::Matrix4x4& GetTransform() const { return transform; }

    BUTIBULLET_API void SetCollisionGroup(const std::uint32_t arg_value);
    BUTIBULLET_API void SetCollisionGroupMask(const std::uint32_t arg_value);

    BUTIBULLET_API void OnDispose(const bool arg_explicitDisposing) override;
    BUTIBULLET_API void OnPrepareStepSimulation() override;
    BUTIBULLET_API void OnAfterStepSimulation() override;

    BUTIBULLET_API TriggerBody();
    BUTIBULLET_API void Initialize();
    BUTIBULLET_API void Initialize(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);

private:
    void RemoveFromBtWorld() override;

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
