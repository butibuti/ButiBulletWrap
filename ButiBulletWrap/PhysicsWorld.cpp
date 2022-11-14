
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

constexpr float internalTimeUnit = 1.0f / 60.0f;
namespace ButiBullet {
float g_speed=1.0f;
void SetGlobalSpeed(const float arg_speed) {
    g_speed = arg_speed;
}
float GetGlobalSpeed() {
    return g_speed;
}


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
ButiBullet::PhysicsWorld::PhysicsWorld(const std::int32_t arg_iteration)
{
    m_iteration = arg_iteration;
    m_iterationTimeUnit = internalTimeUnit/ static_cast<float>(arg_iteration);
}

void ButiBullet::PhysicsWorld::AddPhysicsObject(ButiEngine::Value_ptr< PhysicsObject> arg_vlp_physicsObject)
{
    if (!arg_vlp_physicsObject) return;
    if (arg_vlp_physicsObject->GetPhysicsWorld()) return;
    m_list_vlp_delayAddBodies.Add(arg_vlp_physicsObject);
    arg_vlp_physicsObject->SetPhysicsWorld(value_from_this());
}

void ButiBullet::PhysicsWorld::AddJoint(ButiEngine::Value_ptr< IJoint> arg_vlp_joint)
{
    if (!arg_vlp_joint) return;
    if (arg_vlp_joint->GetPhysicsWorld()) return;
    m_list_vlp_delayAddJoints.Add(arg_vlp_joint);
    arg_vlp_joint->SetPhysicsWorld( value_from_this());
}

void ButiBullet::PhysicsWorld::RemovePhysicsObject(ButiEngine::Value_ptr< PhysicsObject> arg_vlp_physicsObject)
{
    if (!arg_vlp_physicsObject) return;
    if (arg_vlp_physicsObject->GetPhysicsWorld()!= value_from_this()) return;
    arg_vlp_physicsObject->removing = true;
}

void ButiBullet::PhysicsWorld::RemoveJoint(ButiEngine::Value_ptr< IJoint> arg_vlp_joint)
{
    if (!arg_vlp_joint) return;
    if (arg_vlp_joint->GetPhysicsWorld()!= value_from_this()) return;
    arg_vlp_joint->SetIsRemoving(true);
}

bool ButiBullet::PhysicsWorld::Raycast(const ButiEngine::Vector3& arg_origin, const ButiEngine::Vector3& arg_direction, const float arg_maxDistance, const std::uint32_t arg_layerMask, const bool arg_queryTrigger, PhysicsRaycastResult* arg_p_outResult)
{
    btCollisionWorld::ClosestRayResultCallback callback(
        PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_origin),
        PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_origin + arg_direction * arg_maxDistance));
    m_p_btWorld->rayTest(callback.m_rayFromWorld, callback.m_rayToWorld, callback);

    if (arg_p_outResult && callback.hasHit()) {
        arg_p_outResult->physicsObject = reinterpret_cast<ButiEngine::Value_weak_ptr<PhysicsObject>*>(callback.m_collisionObject->getUserPointer())->lock().get();
        arg_p_outResult->point = PhysicsDetail::BulletUtil::btVector3ToVector3(callback.m_hitPointWorld);
        arg_p_outResult->normal = PhysicsDetail::BulletUtil::btVector3ToVector3(callback.m_hitNormalWorld);
        arg_p_outResult->distance = arg_maxDistance * callback.m_closestHitFraction;
    }
    else {
        arg_p_outResult->physicsObject = nullptr;
    }

    return callback.hasHit();
}

void ButiBullet::PhysicsWorld::StepSimulation(const float arg_elapsedSeconds)
{
    UpdateObjectList();
    {
        std::lock_guard lock(m_mtx_sim);
        for (auto& obj : m_list_vlp_physicsObject) {
            obj->OnPrepareStepSimulation();
        }
        for (auto& joint : m_list_vlp_joint) {
            joint->OnPrepareStepSimulation();
        }
    }


    {
        std::lock_guard lock(m_mtx_sim);
        m_p_btWorld->applyGravity();
        m_p_btWorld->stepSimulation(arg_elapsedSeconds, m_iteration, m_iterationTimeUnit * GetGlobalSpeed());

    }


    ProcessContactCommands();
    {
        std::lock_guard lock(m_mtx_sim);
        for (auto& obj : m_list_vlp_physicsObject) {
            obj->OnAfterStepSimulation();
        }
    }
}

void ButiBullet::PhysicsWorld::RenderDebug(RenderingContext* arg_p_context)
{ 
    
}

void ButiBullet::PhysicsWorld::PostBeginContact(ButiEngine::Value_ptr< PhysicsObject > arg_vlp_self, ButiEngine::Value_ptr< PhysicsObject > arg_vlp_other)
{
    if (!arg_vlp_self||!arg_vlp_other) return;

    m_list_contactCommands.push_back({ ContactCommandType::Begin, arg_vlp_self, arg_vlp_other });
}

void ButiBullet::PhysicsWorld::PostEndContact(ButiEngine::Value_ptr< PhysicsObject > arg_vlp_self, ButiEngine::Value_ptr< PhysicsObject > arg_vlp_other)
{
    if (!arg_vlp_self || !arg_vlp_other) return;
    m_list_contactCommands.push_back({ ContactCommandType::End, arg_vlp_self, arg_vlp_other});
}

void ButiBullet::PhysicsWorld::ProcessContactCommands()
{
    std::lock_guard lock(m_mtx_sim);
    if (m_list_contactCommands.GetSize()) {
        for (auto command : m_list_contactCommands) {
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
        m_list_contactCommands.Clear();
    }

    for (auto& obj : m_list_vlp_physicsObject) {
        for (auto& other : obj->GetContactBodies()) {
            obj->OnCollisionStay(other.lock().get(), nullptr);
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
    m_p_btCollisionConfig = new btSoftBodyRigidBodyCollisionConfiguration();

    m_p_btCollisionDispatcher = new btCollisionDispatcher(m_p_btCollisionConfig);

    m_p_btBroadphase = new btDbvtBroadphase();

    m_p_btSolver = new btSequentialImpulseConstraintSolver();

    m_p_btWorld = new btSoftRigidDynamicsWorld(m_p_btCollisionDispatcher, m_p_btBroadphase, m_p_btSolver, m_p_btCollisionConfig, nullptr);

    m_p_btWorld->setGravity(btVector3(0.0f, -9.8f, 0.0f));


    m_p_btGhostPairCallback = new btGhostPairCallback();
    m_p_btWorld->getPairCache()->setInternalGhostPairCallback(m_p_btGhostPairCallback);


    m_p_softBodyWorldInfo = new btSoftBodyWorldInfo();
    m_p_softBodyWorldInfo->air_density = 1.2f;
    m_p_softBodyWorldInfo->water_density = 0;
    m_p_softBodyWorldInfo->water_offset = 0;
    m_p_softBodyWorldInfo->water_normal = btVector3(0.0f, 0.0f, 0.0f);
    m_p_softBodyWorldInfo->m_gravity = m_p_btWorld->getGravity();
    m_p_softBodyWorldInfo->m_broadphase = m_p_btBroadphase;
    m_p_softBodyWorldInfo->m_dispatcher = m_p_btCollisionDispatcher;
    m_p_softBodyWorldInfo->m_sparsesdf.Initialize();
    gContactStartedCallback = ContactStartedCallback;
    gContactEndedCallback = ContactEndedCallback;

}

void ButiBullet::PhysicsWorld::OnDispose(const bool arg_explicitDisposing)
{
    for (auto& obj : m_list_vlp_delayAddBodies) obj->removing = true;
    for (auto& obj : m_list_vlp_delayAddJoints) obj->SetIsRemoving( true);
    for (auto& obj : m_list_vlp_physicsObject) obj->removing = true;
    for (auto& obj : m_list_vlp_joint) obj->SetIsRemoving(true);
    UpdateObjectList();

    delete (m_p_softBodyWorldInfo);
    delete (m_p_btGhostPairCallback);
    delete (m_p_btWorld);
    delete (m_p_btSolver);
    delete (m_p_btBroadphase);
    delete (m_p_btCollisionDispatcher);
    delete (m_p_btCollisionConfig);

}

void ButiBullet::PhysicsWorld::UpdateObjectList()
{
    std::lock_guard lock(m_mtx_sim);
    // Delayed Add
    for (auto& obj : m_list_vlp_delayAddBodies) {
        m_list_vlp_physicsObject.Add(obj);
    }
    for (auto& joint : m_list_vlp_delayAddJoints) {
        m_list_vlp_joint.Add(joint);
    }
    m_list_vlp_delayAddBodies.Clear();
    m_list_vlp_delayAddJoints.Clear();

    for (std::int32_t i = m_list_vlp_physicsObject.GetSize() - 1; i >= 0; i--) {
        auto& obj = m_list_vlp_physicsObject[i];
        if (obj->removing) {
            obj->RemoveFromBtWorld();
            obj->removing = false;
            m_list_vlp_physicsObject.RemoveAt(i);
        }
    }
    for (std::int32_t i = m_list_vlp_joint.GetSize() - 1; i >= 0; i--) {
        auto& joint = m_list_vlp_joint[i];
        if (joint->GetIsRemoving()) {
            joint->RemoveFromBtWorld();
            joint->SetIsRemoving(false);
            m_list_vlp_joint.RemoveAt(i);
        }
    }
}

void ButiBullet::PhysicsWorld::AddObjectInternal(PhysicsObject* arg_p_obj)
{
    switch (arg_p_obj->GetPhysicsObjectType())
    {
    case PhysicsObjectType::RigidBody:
        m_p_btWorld->addRigidBody(reinterpret_cast<RigidBody*>(arg_p_obj)->GetBody());
        break;
    default:

        break;
    }
}

