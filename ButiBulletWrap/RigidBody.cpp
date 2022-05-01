#include"stdafx.h"
#include "RigidBody.h"
#include"BulletUtil.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "LinearMath/btMotionState.h"
#include "PhysicsWorld.h"
#include "PhysicsManager.h"
namespace ButiBullet {

namespace PhysicsDetail {

class SynchronizeMotionState
    : public btDefaultMotionState
{
public:
    ButiBullet::RigidBody* p_owner;

    SynchronizeMotionState(ButiBullet::RigidBody* arg_p_owner, const btTransform& startTrans = btTransform::getIdentity(), const btTransform& centerOfMassOffset = btTransform::getIdentity())
        : btDefaultMotionState(startTrans, centerOfMassOffset)
        , p_owner(arg_p_owner)
    {
    }

    virtual void setWorldTransform(const btTransform& centerOfMassWorldTrans) override
    {
        btDefaultMotionState::setWorldTransform(centerOfMassWorldTrans);
    }
};
}
const ButiEngine::Vector3& RigidBody::GetAngularVelocity() const
{
    return angularVelocity;
}
ButiEngine::Flags<RigidBodyLimitFlags> RigidBody::GetLinearLimits() const
{
    return flg_linearLimits;
}
ButiEngine::Flags<RigidBodyLimitFlags> RigidBody::GetAngularLimits() const
{
    return flg_angularLimits;
}
float RigidBody::GetLinearDamping() const
{
    return linearDamping;
}
float RigidBody::GetAngularDamping() const
{
    return angularDamping;
}
float RigidBody::GetFriction() const
{
    return friction;
}
float RigidBody::GetRestitution() const
{
    return restitution;
}
}
ButiEngine::Value_ptr<ButiBullet::RigidBody> ButiBullet::RigidBody::Create(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape)
{
    auto output=ButiEngine::make_value<RigidBody>();
    output->Initialize(arg_vlp_shape);
    return output;
}

ButiBullet::RigidBody::RigidBody()
    : PhysicsObject(PhysicsObjectType::RigidBody)
    , p_btRigidBody(nullptr)
    , p_btShapeManager()
    , transform()
    , mass(0.0f)
    , scale(1.0f)
    , group(0x00000001)
    , groupMask(0x0000FFFF) 
    , isKinematicObject(false)
    , isAdditionalDamping(false)
    , flg_linearLimits(RigidBodyLimitFlags::None)
    , flg_angularLimits(RigidBodyLimitFlags::None)
    , linearDamping(0.0f)
    , angularDamping(0.0f)
    , friction(0.5)
    , restitution(0.0f)
    , linearVelocity()
    , angularVelocity()
    , appliedCenterForce()
    , appliedCenterImpulse()
    , appliedTorque()
    , appliedTorqueImpulse()
    , modifiedFlags(Modified_All)
{
}

ButiBullet::RigidBody::~RigidBody()
{
    if (p_btRigidBody != nullptr)
    {
        btMotionState* state = p_btRigidBody->getMotionState();
        delete (state);
        delete (p_btRigidBody);
    }
}

void ButiBullet::RigidBody::Initialize()
{
    AttemptAddToActiveWorld();
}

void ButiBullet::RigidBody::Initialize(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape)
{
    AddCollisionShape(arg_vlp_shape);
}

void ButiBullet::RigidBody::SetMass(const float arg_mass)
{
    mass = arg_mass;
    modifiedFlags |= Modified_Mass;
}

void ButiBullet::RigidBody::SetScale(const float arg_scale)
{
    scale = arg_scale;
    Activate();
}

float ButiBullet::RigidBody::GetMass() const
{
    return mass;
}

float ButiBullet::RigidBody::GetScale() const
{
    return scale;
}

void ButiBullet::RigidBody::SetVelocity(const ButiEngine::Vector3& arg_velocity)
{
    linearVelocity = arg_velocity;
    modifiedFlags |= Modified_LinearVelocity;
}

ButiEngine::Vector3 ButiBullet::RigidBody::GetVelocity() const
{
    if (!p_btRigidBody) {
        return linearVelocity;
    }
    else {
        return PhysicsDetail::BulletUtil::btVector3ToVector3(p_btRigidBody->getLinearVelocity());
    }
}

void ButiBullet::RigidBody::SetAngularVelocity(const ButiEngine::Vector3& arg_velocity)
{
    angularVelocity = arg_velocity;
    modifiedFlags |= Modified_AngularVelocity;
}

void ButiBullet::RigidBody::SetLinearLimits(ButiEngine:: Flags<RigidBodyLimitFlags> arg_flags)
{
    flg_linearLimits = arg_flags;
    modifiedFlags |= Modified_LimittFlags;
}

void ButiBullet::RigidBody::SetAngularLimits(ButiEngine::Flags<RigidBodyLimitFlags> arg_flags)
{
    flg_angularLimits = arg_flags;
    modifiedFlags |= Modified_LimittFlags;
}

void ButiBullet::RigidBody::SetLinearDamping(const float arg_damping)
{
    linearDamping = arg_damping;
    modifiedFlags |= Modified_UniformParams;
}

void ButiBullet::RigidBody::SetAngularDamping(const float arg_damping)
{
    angularDamping = arg_damping;
    modifiedFlags |= Modified_UniformParams;
}

void ButiBullet::RigidBody::SetFriction(const float arg_friction)
{
    friction = friction;
    modifiedFlags |= Modified_UniformParams;
}

void ButiBullet::RigidBody::SetRestitution(const float arg_restitution)
{
    restitution = arg_restitution;
    modifiedFlags |= Modified_UniformParams;
}

void ButiBullet::RigidBody::SetIsKinematic(const bool arg_enabled)
{
    isKinematicObject = arg_enabled;
    modifiedFlags |= Modified_ReaddToWorld;
}

void ButiBullet::RigidBody::SetIsAdditionalDamping(const bool arg_enabled)
{
    isAdditionalDamping= arg_enabled;
    modifiedFlags |= Modified_ReaddToWorld;
}

void ButiBullet::RigidBody::SetCollisionGroup(const uint32_t arg_group)
{
    group = arg_group;
    modifiedFlags |= Modified_ReaddToWorld;
}

void ButiBullet::RigidBody::SetCollisionGroupMask(uint32_t arg_groupMask)
{
    groupMask = arg_groupMask;
    modifiedFlags |= Modified_ReaddToWorld;
}

uint32_t ButiBullet::RigidBody::GetCollisionGroup()
{
    return group;
}

uint32_t ButiBullet::RigidBody::GetCollisionGroupMask()
{
    return groupMask;
}

void ButiBullet::RigidBody::SetTransform(const ButiEngine::Matrix4x4& arg_transform)
{
    transform = arg_transform;
    modifiedFlags |= Modified_WorldTransform;
}

void ButiBullet::RigidBody::ApplyForce(const ButiEngine::Vector3& arg_force)
{
    if (arg_force.GetLength() > 0.1) {
        std::int32_t i= 0;
    }
    appliedCenterForce += arg_force;
    modifiedFlags |= Modified_ApplyCenterForce;
    Activate();
}

void ButiBullet::RigidBody::ApplyForce(const ButiEngine::Vector3& arg_force, const ButiEngine::Vector3& arg_localPosition)
{
    ApplyForce(arg_force);
    ApplyTorque(arg_localPosition.GetCross(arg_force * GetLinearFactor()));
}

void ButiBullet::RigidBody::ApplyImpulse(const ButiEngine::Vector3& arg_impulse)
{
    appliedCenterImpulse += arg_impulse;
    modifiedFlags |= Modified_ApplyCenterImpulse;
    Activate();
}

void ButiBullet::RigidBody::ApplyImpulse(const ButiEngine::Vector3& arg_impulse, const ButiEngine::Vector3& arg_localPosition)
{
    // see btRigidBody::applyImpulse
    ApplyImpulse(arg_impulse);
    ApplyTorqueImpulse(arg_localPosition.GetCross( arg_impulse * GetLinearFactor()));
}

void ButiBullet::RigidBody::ApplyTorque(const ButiEngine::Vector3& arg_torque)
{
    appliedTorque += arg_torque;
    modifiedFlags |= Modified_ApplyCenterForce;
}

void ButiBullet::RigidBody::ApplyTorqueImpulse(const ButiEngine::Vector3& arg_torque)
{
    appliedTorqueImpulse += arg_torque;
    modifiedFlags |= Modified_ApplyTorqueImpulse;
}

void ButiBullet::RigidBody::ClearForces()
{
    modifiedFlags |= Modified_ClearForces;
    modifiedFlags &= ~Modified_ApplyCenterForce;
    modifiedFlags &= ~Modified_ApplyCenterImpulse;
    modifiedFlags &= ~Modified_ApplyTorque;
    modifiedFlags &= ~Modified_ApplyTorqueImpulse;
    appliedCenterForce =ButiEngine::Vector3Const::Zero;
    appliedCenterImpulse = ButiEngine::Vector3Const::Zero;
    appliedTorque = ButiEngine::Vector3Const::Zero;
    appliedTorqueImpulse = ButiEngine::Vector3Const::Zero;
}


void ButiBullet::RigidBody::AddCollisionShape(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape)
{
    p_btShapeManager.AddShape(arg_vlp_shape);
}


void ButiBullet::RigidBody::OnPrepareStepSimulation()
{
    PhysicsObject::OnPrepareStepSimulation();


    if ((modifiedFlags & Modified_InitialUpdate) )
    {
        CreateBtRigidBody();
        modifiedFlags &= ~Modified_ReaddToWorld;
    }

    if ((modifiedFlags & Modified_WorldTransform)  | isKinematicObject)
    {
        btTransform l_transform;
        l_transform.setFromOpenGLMatrix(reinterpret_cast<btScalar*>( &transform));
        p_btRigidBody->setWorldTransform(l_transform);

        if (p_btRigidBody )
        {
            p_btRigidBody->getMotionState()->setWorldTransform(l_transform);
        }
    }

    if (p_btRigidBody )
    {
        if ((modifiedFlags & Modified_LimittFlags) )
        {
            p_btRigidBody->setLinearFactor(PhysicsDetail::BulletUtil::Vector3ToBtVector3(GetLinearFactor()));
            p_btRigidBody->setAngularFactor(PhysicsDetail::BulletUtil::Vector3ToBtVector3(GetAngularFactor()));
        }

        // Mass
        if ((modifiedFlags & Modified_Mass) )
        {
            bool isStatic = p_btRigidBody->isStaticObject();
            if (mass)
            {
                btVector3 inertia;
                p_btRigidBody->getCollisionShape()->calculateLocalInertia(mass, inertia);
                p_btRigidBody->setMassProps(mass, inertia);
            }


            if (isStatic != p_btRigidBody->isStaticObject())
            {
                ReaddToWorld();
            }
        }

        if ((modifiedFlags & Modified_LinearVelocity))
        {
            p_btRigidBody->setLinearVelocity(PhysicsDetail::BulletUtil::Vector3ToBtVector3(linearVelocity));
        }
        if ((modifiedFlags & Modified_AngularVelocity) )
        {
            p_btRigidBody->setAngularVelocity(PhysicsDetail::BulletUtil::Vector3ToBtVector3(angularVelocity));
        }
        if ((modifiedFlags & Modified_UniformParams))
        {
            p_btRigidBody->setDamping(linearDamping, angularDamping);
            p_btRigidBody->setFriction(friction);
            p_btRigidBody->setRestitution(restitution);
        }

        if ((modifiedFlags & Modified_ReaddToWorld) )
        {
            ReaddToWorld();
        }

        // clearForces 要求
        if ((modifiedFlags & Modified_ClearForces))
        {
            p_btRigidBody->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
            p_btRigidBody->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
            p_btRigidBody->clearForces();
        }

        if ((modifiedFlags & Modified_ApplyCenterForce) )
        {
            p_btRigidBody->applyCentralForce(PhysicsDetail::BulletUtil::Vector3ToBtVector3(appliedCenterForce));
            appliedCenterForce = ButiEngine:: Vector3Const::Zero;
        }
        if ((modifiedFlags & Modified_ApplyCenterImpulse))
        {
            p_btRigidBody->applyCentralImpulse(PhysicsDetail::BulletUtil::Vector3ToBtVector3(appliedCenterImpulse));
            appliedCenterImpulse = ButiEngine::Vector3Const::Zero;
        }
        if ((modifiedFlags & Modified_ApplyCenterImpulse) )
        {
            p_btRigidBody->applyTorque(PhysicsDetail::BulletUtil::Vector3ToBtVector3(appliedTorque));
            appliedTorque = ButiEngine::Vector3Const::Zero;
        }
        if ((modifiedFlags & Modified_ApplyTorqueImpulse) )
        {
            p_btRigidBody->applyTorqueImpulse(PhysicsDetail::BulletUtil::Vector3ToBtVector3(appliedTorqueImpulse));
            appliedTorqueImpulse =ButiEngine::Vector3Const::Zero;
        }
    }

    // activate 要求
    if ((modifiedFlags & Modified_Activate) )
    {
        p_btRigidBody->activate();
    }

    modifiedFlags = Modified_None;
}

void ButiBullet::RigidBody::OnAfterStepSimulation()
{
    if (!isKinematicObject)
    {
        btTransform l_transform;
        p_btRigidBody->getMotionState()->getWorldTransform(l_transform);
        l_transform.getOpenGLMatrix(reinterpret_cast<btScalar*>( &transform));
    }

    PhysicsObject::OnAfterStepSimulation();
}

void ButiBullet::RigidBody::RemoveFromBtWorld()
{
    GetPhysicsWorld()->GetBtWorld()->removeRigidBody(p_btRigidBody);
}

void ButiBullet::RigidBody::AttemptAddToActiveWorld()
{
}

void ButiBullet::RigidBody::Activate()
{
    modifiedFlags |= Modified_Activate;
}

void ButiBullet::RigidBody::CreateBtRigidBody()
{
    btCollisionShape* shape = p_btShapeManager.GetBtCollisionShape();


    // 各初期プロパティ
    float num = mass * scale;
    float l_friction;
    float l_hitFraction;
    float l_linearDamping;
    float l_angularDamping;
    btVector3 localInertia(0.0f, 0.0f, 0.0f);
    if (isKinematicObject)
    {
        num = 0.0f;
        l_friction = friction;
        l_hitFraction = restitution;
        l_linearDamping = linearDamping;
        l_angularDamping = angularDamping;
    }
    else
    {
        if (num != 0.0f)
        {
            shape->calculateLocalInertia(num, localInertia);
        }
        l_friction = friction;
        l_hitFraction = restitution;
        l_linearDamping = linearDamping;
        l_angularDamping = angularDamping;
    }

    btTransform initialTransform;
    {
        initialTransform.setFromOpenGLMatrix(reinterpret_cast<const btScalar*>(&transform));
        initialTransform.getOrigin().setX(initialTransform.getOrigin().x() * scale);
        initialTransform.getOrigin().setY(initialTransform.getOrigin().y() * scale);
        initialTransform.getOrigin().setZ(initialTransform.getOrigin().z() * scale);
    }

    btMotionState* motionState;
    if (isKinematicObject)
    {
        motionState = new PhysicsDetail::SynchronizeMotionState(this, initialTransform);
    }
    else
    {
        motionState = new PhysicsDetail::SynchronizeMotionState(this, initialTransform);
    }

    // RigidBodyComponent 作成
    btRigidBody::btRigidBodyConstructionInfo bodyInfo(num, motionState, shape, localInertia);
    bodyInfo.m_linearDamping = l_linearDamping;
    bodyInfo.m_angularDamping = l_angularDamping;
    bodyInfo.m_restitution = restitution;
    bodyInfo.m_friction = l_friction;
    bodyInfo.m_additionalDamping = isAdditionalDamping;
    p_btRigidBody = new btRigidBody(bodyInfo);

    if (isKinematicObject)
    {
        // CF_KINEMATIC_OBJECT と DISABLE_DEACTIVATION はセット。決まり事。
        // http://bulletjpn.web.fc2.com/07_RigidBodyDynamics.html
        p_btRigidBody->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT);
        p_btRigidBody->setActivationState( DISABLE_DEACTIVATION);
    }
    else
    {
    }
    p_btRigidBody->setSleepingThresholds(0.0f, 0.0f);

    modifiedFlags |= Modified_Activate;

    p_btRigidBody->setUserPointer(weakAddress());

    AddToWorld();
}

void ButiBullet::RigidBody::SetTransformFromMotionState(const btTransform& arg_transform)
{
}

void ButiBullet::RigidBody::AddToWorld()
{
    if (p_btRigidBody) {
        GetPhysicsWorld()->GetBtWorld()->addRigidBody(p_btRigidBody, group, groupMask);
    }
}

void ButiBullet::RigidBody::ReaddToWorld()
{
    if (p_btRigidBody) {
        GetPhysicsWorld()->GetBtWorld()->removeRigidBody(p_btRigidBody);
        GetPhysicsWorld()->GetBtWorld()->addRigidBody(p_btRigidBody, group, groupMask);
    }
}

ButiEngine::Vector3 ButiBullet::RigidBody::GetLinearFactor() const
{
    return ButiEngine::Vector3(
        flg_linearLimits.hasFlag(RigidBodyLimitFlags::LockedPositionX) ? 0.0f : 1.0f,
        flg_linearLimits.hasFlag(RigidBodyLimitFlags::LockedPositionY) ? 0.0f : 1.0f,
        flg_linearLimits.hasFlag(RigidBodyLimitFlags::LockedPositionZ) ? 0.0f : 1.0f);
}

ButiEngine::Vector3 ButiBullet::RigidBody::GetAngularFactor() const
{
    return ButiEngine::Vector3(
        flg_angularLimits.hasFlag(RigidBodyLimitFlags::LockedRotationX) ? 0.0f : 1.0f,
        flg_angularLimits.hasFlag(RigidBodyLimitFlags::LockedRotationY) ? 0.0f : 1.0f,
        flg_angularLimits.hasFlag(RigidBodyLimitFlags::LockedRotationZ) ? 0.0f : 1.0f);
}
