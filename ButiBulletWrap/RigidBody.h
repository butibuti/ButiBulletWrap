#ifndef BUTIBULLET_RIGIDBODY_H
#define BUTIBULLET_RIGIDBODY_H

#include "Common.h"
#include "PhysicsObject.h"
#include "CollisionShape.h"

namespace ButiBullet {

namespace PhysicsDetail {
class SynchronizeMotionState;
}

class RigidBody: public PhysicsObject,public IRigidBody
{
public:

    BUTIBULLET_API static ButiEngine::Value_ptr<RigidBody> Create(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);

    BUTIBULLET_API RigidBody();
    BUTIBULLET_API ~RigidBody();
    BUTIBULLET_API void Initialize();
    BUTIBULLET_API void Initialize(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);

    BUTIBULLET_API void SetMass(const float arg_mass)override;
    BUTIBULLET_API void SetScale(const float arg_scale)override;
    BUTIBULLET_API float GetMass()const override;
    BUTIBULLET_API float GetScale()const override;

    BUTIBULLET_API void SetVelocity(const ButiEngine::Vector3& arg_velocity)override;
    BUTIBULLET_API ButiEngine::Vector3 GetVelocity() const override;

    BUTIBULLET_API void SetAngularVelocity(const ButiEngine::Vector3& arg_velocity)override;
    BUTIBULLET_API void SetLinearLimits(ButiEngine::Flags<RigidBodyLimitFlags> arg_flags)override;
    BUTIBULLET_API void SetAngularLimits(ButiEngine::Flags<RigidBodyLimitFlags> arg_flags)override;
    BUTIBULLET_API void SetLinearDamping(const float arg_damping)override;
    BUTIBULLET_API void SetAngularDamping(const float arg_damping)override;
    BUTIBULLET_API void SetFriction(const float arg_friction)override;
    BUTIBULLET_API void SetRestitution(const float arg_restitution)override;
    BUTIBULLET_API void SetIsKinematic(const bool arg_enabled)override;
    BUTIBULLET_API void SetIsAdditionalDamping(const bool arg_enabled)override;

    BUTIBULLET_API const ButiEngine::Vector3& GetAngularVelocity()const override;
    BUTIBULLET_API ButiEngine::Flags<RigidBodyLimitFlags>GetLinearLimits()const override;
    BUTIBULLET_API ButiEngine::Flags<RigidBodyLimitFlags>GetAngularLimits()const override;
    BUTIBULLET_API float GetLinearDamping()const override;
    BUTIBULLET_API float GetAngularDamping()const override;
    BUTIBULLET_API float GetFriction()const override;
    BUTIBULLET_API float GetRestitution()const override;

    BUTIBULLET_API bool IsDynamic() const { return !IsStatic() && !IsKinematic(); }
    BUTIBULLET_API bool IsStatic() const { return mass == 0.0f; }
    BUTIBULLET_API bool IsKinematic() const { return isKinematicObject; }
    BUTIBULLET_API bool IsAdditionalDamping() const { return isAdditionalDamping; }

    BUTIBULLET_API void SetCollisionGroup(const uint32_t arg_group)override;
    BUTIBULLET_API void SetCollisionGroupMask(uint32_t arg_groupMask)override;
    BUTIBULLET_API uint32_t GetCollisionGroup()override;
    BUTIBULLET_API uint32_t GetCollisionGroupMask()override;

    BUTIBULLET_API void SetTransform(const ButiEngine::Matrix4x4& arg_transform)override;
    BUTIBULLET_API const ButiEngine::Matrix4x4& GetTransform() const { return transform; }

    BUTIBULLET_API void ApplyForce(const ButiEngine::Vector3& arg_force)override;
    BUTIBULLET_API void ApplyForce(const ButiEngine::Vector3& arg_force, const ButiEngine::Vector3& arg_localPosition)override;

    BUTIBULLET_API void ApplyImpulse(const ButiEngine::Vector3& arg_impulse)override;
    BUTIBULLET_API void ApplyImpulse(const ButiEngine::Vector3& arg_impulse, const ButiEngine::Vector3& arg_localPosition)override;
    BUTIBULLET_API void ApplyTorque(const ButiEngine::Vector3& arg_torque)override;
    BUTIBULLET_API void ApplyTorqueImpulse(const ButiEngine::Vector3& arg_torque)override;
    BUTIBULLET_API void ClearForces()override;
    inline const ButiEngine::Vector3& GetPosition()const override{ return transform.GetPosition(); }
    inline void SetPosition(const ButiEngine::Vector3& arg_pos) override{ transform.SetPosition(arg_pos); Activate(); }

    BUTIBULLET_API void AddCollisionShape(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape);


    BUTIBULLET_API btRigidBody* GetBody() const { return p_btRigidBody; }

protected:
    BUTIBULLET_API void OnPrepareStepSimulation() override;
    BUTIBULLET_API void OnAfterStepSimulation() override;

private:
    void RemoveFromBtWorld() override;
    void AttemptAddToActiveWorld();
    void Activate();
    void CreateBtRigidBody();
    void SetTransformFromMotionState(const btTransform& arg_transform);
    void AddToWorld();
    void ReaddToWorld();
    ButiEngine::Vector3 GetLinearFactor() const;
    ButiEngine::Vector3 GetAngularFactor() const;

    enum ModifiedFlags
    {
        Modified_None = 0x0000,
        Modified_Activate = 0x0001,
        Modified_WorldTransform = 0x0002,
        Modified_ClearForces = 0x0004,
        Modified_Mass = 0x0008,
        Modified_LimittFlags = 0x0040,
        Modified_Colliders = 0x0080,
        Modified_LinearVelocity = 0x1000,
        Modified_AngularVelocity = 0x2000,
        Modified_UniformParams = 0x4000,
        Modified_InitialUpdate = 0x8000,

        Modified_ReaddToWorld = 0x10000,

        Modified_ApplyCenterForce = 0x0010,
        Modified_ApplyCenterImpulse = 0x0020,
        Modified_ApplyTorque = 0x20000,
        Modified_ApplyTorqueImpulse = 0x40000,
        Modified_All = 0xFFFFFFFF,
    };

    btRigidBody* p_btRigidBody;
    PhysicsDetail::BtShapeManager p_btShapeManager;

    ButiEngine::Matrix4x4 transform;


    float mass;
    float scale;
    uint32_t group;	
    uint32_t groupMask;
    bool isKinematicObject;
    bool isAdditionalDamping;

    ButiEngine::Flags<RigidBodyLimitFlags> flg_linearLimits;
    ButiEngine::Flags<RigidBodyLimitFlags> flg_angularLimits;

    // UniformParams
    float linearDamping;
    float angularDamping;
    float friction;
    float restitution;

    ButiEngine::Vector3 linearVelocity;
    ButiEngine::Vector3 angularVelocity;
    ButiEngine::Vector3 appliedCenterForce;
    ButiEngine::Vector3 appliedCenterImpulse;
    ButiEngine::Vector3 appliedTorque;
    ButiEngine::Vector3 appliedTorqueImpulse;

    std::int32_t modifiedFlags;

    friend class PhysicsDetail::SynchronizeMotionState;
    friend class PhysicsWorld;
};

}

#endif // !BUTIBULLET_RIGIDBODY_H
