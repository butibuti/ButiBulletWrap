#include"stdafx.h"
#include "PhysicsWorld.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
#include "BulletDynamics/ConstraintSolver/btUniversalConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHinge2Constraint.h"
#include "BulletDynamics/ConstraintSolver/btGearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btFixedConstraint.h"
#include "Joint.h"
#include "RigidBody.h"
#include "BulletUtil.h"

void ButiBullet::Joint_P2P::RemoveFromPhysicsWorld()
{
    if (!m_vlp_world) return;
    m_vlp_world->RemoveJoint(value_from_this());
}

void ButiBullet::Joint_P2P::RemoveFromBtWorld()
{
    if (!m_vlp_world||!m_p_btConstraint) return;
    m_vlp_world->GetBtWorld()->removeConstraint(m_p_btConstraint);
}

ButiBullet::Joint_P2P::Joint_P2P(ButiEngine::Value_ptr<IRigidBody> arg_vlp_object, const ButiEngine::Vector3& arg_jointPosition, const float arg_durability)
  :m_vlp_world(nullptr), m_removing(false) 
{
	m_p_btConstraint = new btPoint2PointConstraint(*reinterpret_cast<RigidBody*>( arg_vlp_object.get())->p_btRigidBody, PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_jointPosition));
	if(arg_durability>0)
    m_p_btConstraint->setBreakingImpulseThreshold(arg_durability);
}

ButiBullet::Joint_P2P::Joint_P2P(ButiEngine::Value_ptr<IRigidBody> arg_vlp_objectA, const ButiEngine::Vector3& arg_jointPositionA, ButiEngine::Value_ptr<IRigidBody> arg_vlp_objectB, const ButiEngine::Vector3& arg_jointPositionB, const float arg_durability)
 :m_vlp_world(nullptr), m_removing(false) 
{
    m_p_btConstraint = new btPoint2PointConstraint(*reinterpret_cast<RigidBody*>(arg_vlp_objectA.get())->p_btRigidBody, *reinterpret_cast<RigidBody*>(arg_vlp_objectB.get())->p_btRigidBody,
        PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_jointPositionA), PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_jointPositionB));
    if (arg_durability > 0)
        m_p_btConstraint->setBreakingImpulseThreshold(arg_durability);
}


ButiBullet::Joint_P2P::~Joint_P2P()
{
    if (m_p_btConstraint != nullptr)
    {
        delete (m_p_btConstraint);
    }
}


ButiEngine::Value_ptr<ButiBullet::IJoint> ButiBullet::CreateP2PJoint(ButiEngine::Value_ptr<IRigidBody> arg_vlp_object, const ButiEngine::Vector3& arg_jointPosition, const float arg_durability)
{
    return ButiEngine::make_value<Joint_P2P>(arg_vlp_object,arg_jointPosition,arg_durability);
}
ButiEngine::Value_ptr<ButiBullet::IJoint> ButiBullet::CreateP2PJoint(ButiEngine::Value_ptr<IRigidBody> arg_vlp_objectA, const ButiEngine::Vector3& arg_jointPositionA, ButiEngine::Value_ptr<IRigidBody> arg_vlp_objectB, const ButiEngine::Vector3& arg_jointPositionB, const float arg_durability)
{
    return ButiEngine::make_value<Joint_P2P>(arg_vlp_objectA, arg_jointPositionA, arg_vlp_objectB, arg_jointPositionB, arg_durability);
}


void ButiBullet::Joint_spring::SetBodyA(ButiEngine::Value_ptr<IRigidBody> arg_vlp_body, const ButiEngine::Matrix4x4& arg_localJunctionPoint)
{
    m_vlp_bodyA = arg_vlp_body;
    m_localJunctionPointA = arg_localJunctionPoint;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::SetBodyB(ButiEngine::Value_ptr<IRigidBody> arg_vlp_body, const ButiEngine::Matrix4x4& arg_localJunctionPoint)
{
    m_vlp_bodyB = arg_vlp_body;
    m_localJunctionPointB = arg_localJunctionPoint;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::SetLinearLowerLimit(const ButiEngine::Vector3& arg_linearLower)
{
    m_linearLowerLimit = arg_linearLower;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::SetLinearUpperLimit(const ButiEngine::Vector3& arg_linearUpper)
{
    m_linearUpperLimit = arg_linearUpper;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::SetAngularLowerLimit(const ButiEngine::Vector3& arg_angularLower)
{
    m_angularLowerLimit = arg_angularLower;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::SetAngularUpperLimit(const ButiEngine::Vector3& arg_angularUpper)
{
    m_angularUpperLimit = arg_angularUpper;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::SetLinearStiffness(const ButiEngine::Vector3& arg_value)
{
    m_linearStiffness = arg_value;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::SetAngularStiffness(const ButiEngine::Vector3& arg_value)
{
    m_angularStiffness = arg_value;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::SetDurability(const float arg_durability)
{
    m_durability = arg_durability;
    m_isParameterChanged = true;
}

void ButiBullet::Joint_spring::OnPrepareStepSimulation()
{
    if (!m_vlp_bodyA || !m_vlp_bodyB) return;

    if (m_isParameterChanged &&m_p_btDofSpringConstraint) {
        GetPhysicsWorld()->GetBtWorld()->removeConstraint(m_p_btDofSpringConstraint);
        delete m_p_btDofSpringConstraint;
        m_p_btDofSpringConstraint = nullptr;
        m_isParameterChanged = false;
    }

    if (!m_p_btDofSpringConstraint)
    {
        m_p_btDofSpringConstraint = new btGeneric6DofSpringConstraint(
            *m_vlp_bodyA->GetBody(), *m_vlp_bodyB->GetBody(), 
            PhysicsDetail::BulletUtil::Matrix4x4ToBtTransform(m_localJunctionPointA),
            PhysicsDetail::BulletUtil::Matrix4x4ToBtTransform(m_localJunctionPointB),
            true);

        m_p_btDofSpringConstraint->setLinearLowerLimit(PhysicsDetail::BulletUtil::Vector3ToBtVector3(m_linearLowerLimit.GetMin(m_linearUpperLimit)));
        m_p_btDofSpringConstraint->setLinearUpperLimit(PhysicsDetail::BulletUtil::Vector3ToBtVector3(m_linearLowerLimit.GetMax(m_linearUpperLimit)));

        m_p_btDofSpringConstraint->setAngularLowerLimit(PhysicsDetail::BulletUtil::Vector3ToBtVector3(m_angularLowerLimit.GetMin(m_angularUpperLimit)));
        m_p_btDofSpringConstraint->setAngularUpperLimit(PhysicsDetail::BulletUtil::Vector3ToBtVector3(m_angularLowerLimit.GetMax(m_angularUpperLimit)));

        if (m_linearStiffness.x != 0.0f)
        {
            m_p_btDofSpringConstraint->enableSpring(0, true);
            m_p_btDofSpringConstraint->setStiffness(0, m_linearStiffness.x);
        }

        if (m_linearStiffness.y != 0.0f)
        {
            m_p_btDofSpringConstraint->enableSpring(1, true);
            m_p_btDofSpringConstraint->setStiffness(1, m_linearStiffness.y);
        }

        if (m_linearStiffness.z != 0.0f)
        {
            m_p_btDofSpringConstraint->enableSpring(2, true);
            m_p_btDofSpringConstraint->setStiffness(2, m_linearStiffness.z);
        }

        m_p_btDofSpringConstraint->enableSpring(3, true);	m_p_btDofSpringConstraint->setStiffness(3, m_angularStiffness.x);
        m_p_btDofSpringConstraint->enableSpring(4, true);	m_p_btDofSpringConstraint->setStiffness(4, m_angularStiffness.y);
        m_p_btDofSpringConstraint->enableSpring(5, true);	m_p_btDofSpringConstraint->setStiffness(5, m_angularStiffness.z);

        m_p_btDofSpringConstraint->setEquilibriumPoint();
        if (m_durability>0.0f) {
            m_p_btDofSpringConstraint->setBreakingImpulseThreshold(m_durability);
        }
        GetPhysicsWorld()->GetBtWorld()->addConstraint(m_p_btDofSpringConstraint);
        m_isParameterChanged = false;
    }

}


ButiBullet::Joint_spring::Joint_spring() :m_p_btDofSpringConstraint(nullptr), m_removing(false),m_isParameterChanged(false)
{
}
ButiBullet::Joint_spring::~Joint_spring()
{
    if (m_p_btDofSpringConstraint) {
        delete m_p_btDofSpringConstraint;
        m_p_btDofSpringConstraint = nullptr;
    }
}

ButiBullet::Joint_spring::Joint_spring(ButiEngine::Value_ptr<IRigidBody> arg_vlp_bodyA, const ButiEngine::Matrix4x4& arg_localJunctionPointA, ButiEngine::Value_ptr<IRigidBody> arg_vlp_bodyB, const ButiEngine::Matrix4x4& arg_localJunctionPointB, const float arg_durability)
    : m_p_btDofSpringConstraint(nullptr), m_removing(false), m_isParameterChanged(false)
{
    m_vlp_bodyA = arg_vlp_bodyA;
    m_localJunctionPointA = arg_localJunctionPointA;
    m_vlp_bodyB = arg_vlp_bodyB;
    m_localJunctionPointB = arg_localJunctionPointB;
    m_durability = arg_durability;
}

void ButiBullet::Joint_spring::RemoveFromBtWorld()
{
    if(m_p_btDofSpringConstraint)
    GetPhysicsWorld()->GetBtWorld()->removeConstraint(m_p_btDofSpringConstraint);
}

ButiEngine::Value_ptr<ButiBullet::Joint_spring> ButiBullet::CreateSpringJoint(ButiEngine::Value_ptr<IRigidBody> arg_vlp_objectA, const ButiEngine::Matrix4x4& arg_jointPositionA, ButiEngine::Value_ptr<IRigidBody> arg_vlp_objectB, const ButiEngine::Matrix4x4& arg_jointPositionB, const float arg_durability)
{
    return ButiEngine::make_value<Joint_spring>(arg_vlp_objectA, arg_jointPositionA, arg_vlp_objectB, arg_jointPositionB,arg_durability);
}