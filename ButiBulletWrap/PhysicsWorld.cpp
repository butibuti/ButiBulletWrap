
#include"stdafx.h"
#pragma warning(disable: 5033)	
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"


#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"

#include "LinearMath/btDefaultMotionState.h"

//bullet multithread
//#include"../examples/MultiThreading/b3Win32ThreadSupport.h"

#include "PhysicsObject.h"
#include "RigidBody.h"
#include "Joint.h"
#include "PhysicsWorld.h"
#include "BulletUtil.h"



extern ContactStartedCallback gContactStartedCallback;
extern ContactEndedCallback gContactEndedCallback;

namespace ButiBullet {
static void ContactStartedCallback(btPersistentManifold* const& manifold)
{
    const auto* bodyA = static_cast<const btCollisionObject*>(manifold->getBody0());
    const auto* bodyB = static_cast<const btCollisionObject*>(manifold->getBody1());
    auto ownerA = reinterpret_cast<ButiEngine::Value_weak_ptr<PhysicsObject>*>(bodyA->getUserPointer())->lock();
    auto ownerB = reinterpret_cast<ButiEngine::Value_weak_ptr<PhysicsObject>*>(bodyB->getUserPointer())->lock();

    if (ownerA && ownerB && ownerA->GetPhysicsObjectType() == PhysicsObjectType::RigidBody && ownerB->GetPhysicsObjectType() == PhysicsObjectType::RigidBody)
    {
        auto world = ownerA->GetPhysicsWorld();
        world->PostBeginContact(ownerA, ownerB);
    }
}

static void ContactEndedCallback(btPersistentManifold* const& manifold)
{
    const auto* bodyA = static_cast<const btCollisionObject*>(manifold->getBody0());
    const auto* bodyB = static_cast<const btCollisionObject*>(manifold->getBody1());
    auto ownerA = reinterpret_cast<ButiEngine::Value_weak_ptr<PhysicsObject>*>(bodyA->getUserPointer())->lock();
    auto ownerB = reinterpret_cast<ButiEngine::Value_weak_ptr<PhysicsObject>*>(bodyB->getUserPointer())->lock();
    if (ownerA && ownerB &&ownerA->GetPhysicsObjectType() == PhysicsObjectType::RigidBody &&ownerB->GetPhysicsObjectType() == PhysicsObjectType::RigidBody) 
    {
        auto world = ownerA->GetPhysicsWorld();
        world->PostEndContact(ownerA, ownerB);
    }
}
}


void ButiBullet::PhysicsWorld::AddPhysicsObject(ButiEngine::Value_ptr< PhysicsObject> arg_vlp_physicsObject)
{
    if (!arg_vlp_physicsObject) return;
    if (arg_vlp_physicsObject->GetPhysicsWorld()) return;
    list_vlp_delayAddBodies.Add(arg_vlp_physicsObject);
    arg_vlp_physicsObject->SetPhysicsWorld(value_from_this());
}

void ButiBullet::PhysicsWorld::AddJoint(ButiEngine::Value_ptr< Joint> arg_vlp_joint)
{
    if (!arg_vlp_joint) return;
    if (arg_vlp_joint) return;
    list_vlp_delayAddJoints.Add(arg_vlp_joint);
    arg_vlp_joint->vlp_world = value_from_this();
}

void ButiBullet::PhysicsWorld::RemovePhysicsObject(ButiEngine::Value_ptr< PhysicsObject> arg_vlp_physicsObject)
{
    if (!arg_vlp_physicsObject) return;
    if (arg_vlp_physicsObject->GetPhysicsWorld()!= value_from_this()) return;
    arg_vlp_physicsObject->removing = true;
}

void ButiBullet::PhysicsWorld::RemoveJoint(ButiEngine::Value_ptr< Joint> arg_vlp_joint)
{
    if (arg_vlp_joint) return;
    if (arg_vlp_joint->GetPhysicsWorld()!= value_from_this()) return;
    arg_vlp_joint->removing = true;
}

bool ButiBullet::PhysicsWorld::Raycast(const ButiEngine::Vector3& arg_origin, const ButiEngine::Vector3& arg_direction, const float arg_maxDistance, const std::uint32_t arg_layerMask, const bool arg_queryTrigger, PhysicsRaycastResult* arg_p_outResult)
{
    btCollisionWorld::ClosestRayResultCallback callback(
        PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_origin),
        PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_origin + arg_direction * arg_maxDistance));
    p_btWorld->rayTest(callback.m_rayFromWorld, callback.m_rayToWorld, callback);

    if (arg_p_outResult && callback.hasHit()) {
        arg_p_outResult->physicsObject = reinterpret_cast<ButiEngine::Value_weak_ptr<PhysicsObject>*>(callback.m_collisionObject->getUserPointer())->lock().get();
        arg_p_outResult->point = PhysicsDetail::BulletUtil::btVector3ToVector3(callback.m_hitPointWorld);
        arg_p_outResult->normal = PhysicsDetail::BulletUtil::btVector3ToVector3(callback.m_hitNormalWorld);
        arg_p_outResult->distance = arg_maxDistance * callback.m_closestHitFraction;
    }

    return callback.hasHit();
}

void ButiBullet::PhysicsWorld::StepSimulation(const float arg_elapsedSeconds)
{
    std::lock_guard lock(mtx_sim);
    UpdateObjectList();
    for (auto& obj : list_vlp_physicsObject) {
        obj->OnPrepareStepSimulation();
    }

    const float internalTimeUnit = 1.0f / 60.0f;




    //p_btWorld->applyGravity();
    const float iteration = 2.0f;   
    p_btWorld->stepSimulation(arg_elapsedSeconds, 2, internalTimeUnit / iteration);


    ProcessContactCommands();

    for (auto& obj : list_vlp_physicsObject) {
        obj->OnAfterStepSimulation();
    }
}

void ButiBullet::PhysicsWorld::RenderDebug(RenderingContext* arg_p_context)
{ 
    
}

void ButiBullet::PhysicsWorld::PostBeginContact(ButiEngine::Value_ptr< PhysicsObject > arg_vlp_self, ButiEngine::Value_ptr< PhysicsObject > arg_vlp_other)
{
    if (!arg_vlp_self||!arg_vlp_other) return;

    vec_contactCommands.push_back({ ContactCommandType::Begin, arg_vlp_self, arg_vlp_other });
}

void ButiBullet::PhysicsWorld::PostEndContact(ButiEngine::Value_ptr< PhysicsObject > arg_vlp_self, ButiEngine::Value_ptr< PhysicsObject > arg_vlp_other)
{
    if (!arg_vlp_self || !arg_vlp_other) return;
    vec_contactCommands.push_back({ ContactCommandType::End, arg_vlp_self, arg_vlp_other});
}

void ButiBullet::PhysicsWorld::ProcessContactCommands()
{
    if (!vec_contactCommands.empty()) {
        for (auto command : vec_contactCommands) {
            switch (command.type)
            {
            case ContactCommandType::Begin:
                command.self->BeginContact(command.other);
                command.other->BeginContact(command.self);
                break;
            case ContactCommandType::End:
                command.self->EndContact(command.other);
                command.other->EndContact(command.self);
                break;
            default:

                break;
            }
        }
        vec_contactCommands.clear();
    }

    for (auto& obj : list_vlp_physicsObject) {
        for (auto& other : obj->list_vlp_contactBodies) {
            obj->OnCollisionStay(other.get(), nullptr);
            other->OnCollisionStay(obj.get(), nullptr);
        }
    }
}


ButiBullet::PhysicsWorld::~PhysicsWorld()
{
}

void ButiBullet::PhysicsWorld::Initialize()
{
    btAlignedAllocSetCustom(&ButiMemorySystem::Allocator::allocate_large, &ButiMemorySystem::Allocator::deallocate_bt);
    btAlignedAllocSetCustomAligned(&ButiMemorySystem::Allocator::allocate_customAlign, &ButiMemorySystem::Allocator::deallocate_bt);

    std::int32_t maxNumOutstandingTasks = 4;

#ifdef BUTIENGINE_USE_PARALLEL
    m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
        "collision",
        processCollisionTask,
        createCollisionLocalStoreMemory,
        maxNumOutstandingTasks));
#endif

    btDefaultCollisionConstructionInfo defaultCollisionConstructionInfo;

    // ソフトボディ使うとき
    p_btCollisionConfig = new btSoftBodyRigidBodyCollisionConfiguration();

    p_btCollisionDispatcher = new btCollisionDispatcher(p_btCollisionConfig);

    p_btBroadphase = new btDbvtBroadphase();

    p_btSolver = new btSequentialImpulseConstraintSolver();

    p_btWorld = new btSoftRigidDynamicsWorld(p_btCollisionDispatcher, p_btBroadphase, p_btSolver, p_btCollisionConfig, nullptr);

    p_btWorld->setGravity(btVector3(0.0f, 0.0f, 0.0f));


    p_btGhostPairCallback = new btGhostPairCallback();
    p_btWorld->getPairCache()->setInternalGhostPairCallback(p_btGhostPairCallback);


    p_softBodyWorldInfo = new btSoftBodyWorldInfo();
    p_softBodyWorldInfo->air_density = 1.2f;
    p_softBodyWorldInfo->water_density = 0;
    p_softBodyWorldInfo->water_offset = 0;
    p_softBodyWorldInfo->water_normal = btVector3(0.0f, 0.0f, 0.0f);
    p_softBodyWorldInfo->m_gravity = p_btWorld->getGravity();
    p_softBodyWorldInfo->m_broadphase = p_btBroadphase;
    p_softBodyWorldInfo->m_dispatcher = p_btCollisionDispatcher;
    p_softBodyWorldInfo->m_sparsesdf.Initialize();
    gContactStartedCallback = ContactStartedCallback;
    gContactEndedCallback = ContactEndedCallback;

}

void ButiBullet::PhysicsWorld::OnDispose(const bool arg_explicitDisposing)
{
    for (auto& obj : list_vlp_delayAddBodies) obj->removing = true;
    for (auto& obj : list_vlp_delayAddJoints) obj->removing = true;
    for (auto& obj : list_vlp_physicsObject) obj->removing = true;
    for (auto& obj : list_vlp_joint) obj->removing = true;
    UpdateObjectList();

    delete (p_softBodyWorldInfo);
    delete (p_btGhostPairCallback);
    delete (p_btWorld);
    delete (p_btSolver);
    delete (p_btBroadphase);
    delete (p_btCollisionDispatcher);
    delete (p_btCollisionConfig);

}

void ButiBullet::PhysicsWorld::UpdateObjectList()
{
    // Delayed Add
    for (auto& obj : list_vlp_delayAddBodies) {
        list_vlp_physicsObject.Add(obj);
    }
    for (auto& obj : list_vlp_delayAddJoints) {
        list_vlp_joint.Add(obj);
    }
    list_vlp_delayAddBodies.Clear();
    list_vlp_delayAddJoints.Clear();

    for (std::int32_t i = list_vlp_physicsObject.GetSize() - 1; i >= 0; i--) {
        auto& obj = list_vlp_physicsObject[i];
        if (obj->removing) {
            obj->RemoveFromBtWorld();
            obj->removing = false;
            list_vlp_physicsObject.RemoveAt(i);
        }
    }
    for (std::int32_t i = list_vlp_joint.GetSize() - 1; i >= 0; i--) {
        auto& obj = list_vlp_joint[i];
        if (obj->removing) {
            obj->RemoveFromBtWorld();
            obj->removing = false;
            list_vlp_joint.RemoveAt(i);
        }
    }
}

void ButiBullet::PhysicsWorld::AddObjectInternal(PhysicsObject* arg_p_obj)
{
    switch (arg_p_obj->GetPhysicsObjectType())
    {
    case PhysicsObjectType::RigidBody:
        p_btWorld->addRigidBody(static_cast<RigidBody*>(arg_p_obj)->GetBody());
        break;
    default:

        break;
    }
}

ButiEngine::Value_ptr<ButiBullet::SpringJoint> ButiBullet::SpringJoint::Create()
{
    return ButiEngine::make_value<SpringJoint>();
}

void ButiBullet::SpringJoint::SetBodyA(ButiEngine::Value_ptr<RigidBody> arg_p_body, const ButiEngine::Matrix4x4& arg_localJunctionPoint)
{
    vlp_bodyA = arg_p_body;
    localJunctionPointA = arg_localJunctionPoint;
}

void ButiBullet::SpringJoint::SetBodyB(ButiEngine::Value_ptr<RigidBody> arg_p_body, const ButiEngine::Matrix4x4& arg_localJunctionPoint)
{
    vlp_bodyB = arg_p_body;
    localJunctionPointB = arg_localJunctionPoint;
}

void ButiBullet::SpringJoint::SetLinearLowerLimit(const ButiEngine::Vector3& arg_linearLower)
{
    linearLowerLimit = arg_linearLower;
}

void ButiBullet::SpringJoint::SetLinearUpperLimit(const ButiEngine::Vector3& arg_linearUpper)
{
    linearUpperLimit = arg_linearUpper;
}

void ButiBullet::SpringJoint::SetAngularLowerLimit(const ButiEngine::Vector3& arg_angularLower)
{
    angularLowerLimit = arg_angularLower;
}

void ButiBullet::SpringJoint::SetAngularUpperLimit(const ButiEngine::Vector3& arg_angularUpper)
{
    angularUpperLimit = arg_angularUpper;
}

void ButiBullet::SpringJoint::SetLinearStiffness(const ButiEngine::Vector3& arg_value)
{
    linearStiffness = arg_value;
}

void ButiBullet::SpringJoint::SetAngularStiffness(const ButiEngine::Vector3& arg_value)
{
    angularStiffness = arg_value;
}

void ButiBullet::SpringJoint::OnDispose(const bool arg_explicitDisposing)
{
    delete p_btDofSpringConstraint;
}

void ButiBullet::SpringJoint::OnPrepareStepSimulation()
{
    if (!vlp_bodyA || vlp_bodyB) return;

    if (!p_btDofSpringConstraint)
    {
        p_btDofSpringConstraint = new btGeneric6DofSpringConstraint(
            *vlp_bodyA->GetBody(), *vlp_bodyB->GetBody(),
            PhysicsDetail::BulletUtil::Matrix4x4ToBtTransform(localJunctionPointA),
            PhysicsDetail::BulletUtil::Matrix4x4ToBtTransform(localJunctionPointB),
            true);

        p_btDofSpringConstraint->setLinearLowerLimit(PhysicsDetail::BulletUtil::Vector3ToBtVector3(linearLowerLimit.GetMin(linearUpperLimit)));
        p_btDofSpringConstraint->setLinearUpperLimit(PhysicsDetail::BulletUtil::Vector3ToBtVector3(linearLowerLimit.GetMax(linearUpperLimit)));

        p_btDofSpringConstraint->setAngularLowerLimit(PhysicsDetail::BulletUtil::Vector3ToBtVector3(angularLowerLimit.GetMin(angularUpperLimit)));
        p_btDofSpringConstraint->setAngularUpperLimit(PhysicsDetail::BulletUtil::Vector3ToBtVector3(angularLowerLimit.GetMax(angularUpperLimit)));

        if (linearStiffness.x != 0.0f)
        {
            p_btDofSpringConstraint->enableSpring(0, true);
            p_btDofSpringConstraint->setStiffness(0, linearStiffness.x);
        }

        if (linearStiffness.y != 0.0f)
        {
            p_btDofSpringConstraint->enableSpring(1, true);
            p_btDofSpringConstraint->setStiffness(1, linearStiffness.y);
        }

        if (linearStiffness.z != 0.0f)
        {
            p_btDofSpringConstraint->enableSpring(2, true);
            p_btDofSpringConstraint->setStiffness(2, linearStiffness.z);
        }

        p_btDofSpringConstraint->enableSpring(3, true);	p_btDofSpringConstraint->setStiffness(3, angularStiffness.x);
        p_btDofSpringConstraint->enableSpring(4, true);	p_btDofSpringConstraint->setStiffness(4, angularStiffness.y);
        p_btDofSpringConstraint->enableSpring(5, true);	p_btDofSpringConstraint->setStiffness(5, angularStiffness.z);

        p_btDofSpringConstraint->setEquilibriumPoint();

        if (p_btDofSpringConstraint) {
            GetPhysicsWorld()->GetBtWorld()->addConstraint(p_btDofSpringConstraint);
        }
    }

}

void ButiBullet::SpringJoint::OnAfterStepSimulation()
{
}

ButiBullet::SpringJoint::SpringJoint()
    : PhysicsObject(PhysicsObjectType::Joint)
    , p_btDofSpringConstraint(nullptr)
{
}

void ButiBullet::SpringJoint::Initialize()
{
}

void ButiBullet::SpringJoint::RemoveFromBtWorld()
{
}
