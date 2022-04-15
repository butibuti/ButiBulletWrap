#ifndef BUTIBULLET_COMMON_H
#define BUTIBULLET_COMMON_H
#include"ButiMath/ButiMath.h"
#include"ButiUtil/ButiUtil/Flag.h"
#include"ButiMemorySystem/ButiMemorySystem/ButiList.h"
#include"ButiMemorySystem/ButiMemorySystem/ButiPtr.h"
class btTransform;
class btDynamicsWorld;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
struct btDbvtBroadphase;
class btAxisSweep3;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;
class btSoftRigidDynamicsWorld;
class btCollisionShape;
class btCompoundShape;
class btRigidBody;
class btSoftBody;
class btGhostObject;
class btGhostPairCallback;
class btTypedConstraint;
class btTriangleIndexVertexArray;
struct btSoftBodyWorldInfo;

class btCollisionObject;
class btManifoldPoint;
struct btCollisionObjectWrapper;
class btGeneric6DofSpringConstraint;

#ifdef BUTIBULLETWRAP_EXPORTS
#define BUTIBULLET_API __declspec(dllexport)
#else
#define BUTIBULLET_API __declspec(dllimport)
#endif
namespace ButiBullet {

class PhysicsWorld;
class PhysicsObject;
class Joint;
class RigidBody;
class ContactPoint;
class CollisionShape;

namespace PhysicsDetail {
class IPhysicsObjectEventListener
{
protected:
    virtual void OnBeforeStepSimulation_Deprecated() = 0;
    virtual void OnAfterStepSimulation() = 0;
    virtual void OnCollisionEnter(PhysicsObject* otherObject, ContactPoint* contact) = 0;
    virtual void OnCollisionLeave(PhysicsObject* otherObject, ContactPoint* contact) = 0;
    virtual void OnCollisionStay(PhysicsObject* otherObject, ContactPoint* contact) = 0;

    friend class PhysicsObject;
};

}

enum class RigidBodyLimitFlags
{
    // 制限なし
    None = 0x0000,

    // X 軸の平行移動制限 
    LockedPositionX = 0x0001,

    // Y 軸の平行移動制限 
    LockedPositionY = 0x0002,

    // Z 軸の平行移動制限
    LockedPositionZ = 0x0004,

    // X 軸の回転制限
    LockedRotationX = 0x0010,

    // Y 軸の回転制限
    LockedRotationY = 0x0020,

    // Z 軸の回転制限
    LockedRotationZ = 0x0040,

    // 平行移動制限 
    LockedPosition = LockedPositionX | LockedPositionY | LockedPositionZ,

    // 回転制限
    LockedRotation = LockedRotationX | LockedRotationY | LockedRotationZ,

    // 運動制限
    LockedAll = LockedPosition | LockedRotation,
};

class IRigidBody {
public:
    virtual void SetMass(const float arg_mass) = 0;
    virtual void SetScale(const float arg_scale)=0;
    virtual float GetMass()const =0;
    virtual float GetScale()const =0;
    virtual void SetVelocity(const ButiEngine::Vector3& arg_velocity) = 0;
    virtual ButiEngine::Vector3 GetVelocity() const = 0;
    virtual void SetAngularVelocity(const ButiEngine::Vector3& arg_velocity) = 0;
    virtual void SetLinearLimits(ButiEngine::Flags<RigidBodyLimitFlags> arg_flags) = 0;
    virtual void SetAngularLimits(ButiEngine::Flags<RigidBodyLimitFlags> arg_flags) = 0;
    virtual void SetLinearDamping(const float arg_damping) = 0;
    virtual void SetAngularDamping(const float arg_damping) = 0;
    virtual void SetFriction(const float arg_friction) = 0;
    virtual void SetRestitution(const float arg_restitution) = 0;
    virtual void SetIsKinematic(const bool arg_enabled) = 0;
    virtual void SetIsAdditionalDamping(const bool arg_enabled) = 0;
    virtual bool IsDynamic() const = 0;
    virtual bool IsStatic() const = 0;
    virtual bool IsKinematic() const = 0;
    virtual bool IsAdditionalDamping() const = 0;
    virtual void SetCollisionGroup(const uint32_t arg_group) = 0;
    virtual void SetCollisionGroupMask(uint32_t arg_groupMask) = 0;
    virtual void SetTransform(const ButiEngine::Matrix4x4& arg_transform) = 0;
    virtual const ButiEngine::Matrix4x4& GetTransform() const = 0;
    virtual void ApplyForce(const ButiEngine::Vector3& arg_force) = 0;
    virtual void ApplyForce(const ButiEngine::Vector3& arg_force, const ButiEngine::Vector3& arg_localPosition) = 0;
    virtual void ApplyImpulse(const ButiEngine::Vector3& arg_impulse) = 0;
    virtual void ApplyImpulse(const ButiEngine::Vector3& arg_impulse, const ButiEngine::Vector3& arg_localPosition) = 0;
    virtual void ApplyTorque(const ButiEngine::Vector3& arg_torque) = 0;
    virtual void ApplyTorqueImpulse(const ButiEngine::Vector3& arg_torque) = 0;
    virtual void ClearForces() = 0;
    virtual const ButiEngine::Vector3& GetPosition()const = 0;
    virtual void SetPosition(const ButiEngine::Vector3& arg_pos) = 0;

    virtual const ButiEngine::Vector3& GetAngularVelocity()const =0;
    virtual ButiEngine::Flags<RigidBodyLimitFlags>GetLinearLimits()const =0;
    virtual ButiEngine::Flags<RigidBodyLimitFlags>GetAngularLimits()const =0;
    virtual float GetLinearDamping()const =0;
    virtual float GetAngularDamping()const =0;
    virtual float GetFriction()const =0;
    virtual float GetRestitution()const =0;
    virtual uint32_t GetCollisionGroup()=0;
    virtual uint32_t GetCollisionGroupMask()=0;

    //ButiScriptからのアクセス用のメソッド 全体をValue_ptrで統一したら解消
    void ApplyForce_s(const ButiEngine::Vector3 arg_force) {
        ApplyForce(arg_force);
    }
    //ButiScriptからのアクセス用のメソッド 全体をValue_ptrで統一したら解消
    void ApplyTorque_s(const ButiEngine::Vector3 arg_torque) {
        ApplyTorque(arg_torque);
    }
    //ButiScriptからのアクセス用のメソッド 全体をValue_ptrで統一したら解消
    void ApplyImpulse_s(const ButiEngine::Vector3 arg_impulse) {
        ApplyImpulse(arg_impulse);
    }
    //ButiScriptからのアクセス用のメソッド 全体をValue_ptrで統一したら解消
    void ApplyTorqueImpulse_s(const ButiEngine::Vector3 arg_torque) {
        ApplyTorqueImpulse(arg_torque);
    }
    //ButiScriptからのアクセス用のメソッド 全体をValue_ptrで統一したら解消
    void SetPosition_s(const ButiEngine::Vector3 arg_pos) {
        SetPosition(arg_pos);
    }
    //ButiScriptからのアクセス用のメソッド 全体をValue_ptrで統一したら解消
    void SetAngularVelocity_s(const ButiEngine::Vector3 arg_velocity) {
        SetAngularVelocity(arg_velocity);
    }
    //ButiScriptからのアクセス用のメソッド 全体をValue_ptrで統一したら解消
    void SetVelocity_s(const ButiEngine::Vector3 arg_velocity) {
        SetVelocity(arg_velocity);
    }

};
}
#endif // !BUTIBULLET_COMMON_H
