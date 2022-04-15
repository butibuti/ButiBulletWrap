#ifndef BUTIBULLET_PHYSICSWORLD_H
#define BUTIBULLET_PHYSICSWORLD_H
#include"Common.h"
#include<mutex>
#include<vector>
#include"PhysicsObject.h"
#include"ButiMemorySystem/ButiMemorySystem/ButiPtr.h"
namespace ButiBullet {

class RenderingContext;
class PhysicsObject;
class RigidBody;
class PhysicsJoint;

struct PhysicsRaycastResult
{
    PhysicsObject* physicsObject;
    ButiEngine::Vector3 point;
    ButiEngine::Vector3 normal;
    float distance;
};


class PhysicsWorld:public ButiEngine::enable_value_from_this<PhysicsWorld>
{
public:
    PhysicsWorld(){}
    PhysicsWorld(const PhysicsWorld& arg_other){}
    BUTIBULLET_API virtual ~PhysicsWorld();

    BUTIBULLET_API void AddPhysicsObject(ButiEngine::Value_ptr< PhysicsObject >arg_vlp_physicsObject);
    BUTIBULLET_API void AddJoint(ButiEngine::Value_ptr< Joint> arg_vlp_joint);

    BUTIBULLET_API void RemovePhysicsObject(ButiEngine::Value_ptr< PhysicsObject > arg_vlp_physicsObject);
    BUTIBULLET_API void RemoveJoint(ButiEngine::Value_ptr< Joint> arg_vlp_joint);

    BUTIBULLET_API bool Raycast(const ButiEngine::Vector3& arg_origin, const ButiEngine::Vector3& arg_direction,const float arg_maxDistance,const std::uint32_t arg_layerMask,const bool arg_queryTrigger , PhysicsRaycastResult* arg_p_outResult = nullptr);
    BUTIBULLET_API bool Raycast(const ButiEngine::Vector3& arg_origin, const ButiEngine::Vector3& arg_direction,const float arg_maxDistance,const std::uint32_t arg_layerMask, PhysicsRaycastResult* arg_p_outResult = nullptr) { return Raycast(arg_origin, arg_direction, arg_maxDistance, arg_layerMask, false, arg_p_outResult); }


    btSoftRigidDynamicsWorld* GetBtWorld() { return p_btWorld; }
    btSoftBodyWorldInfo* GetSoftBodyWorldInfo() const { return p_softBodyWorldInfo; }
    BUTIBULLET_API void StepSimulation(const float arg_elapsedSeconds);
    BUTIBULLET_API void RenderDebug(RenderingContext* arg_p_context);

    BUTIBULLET_API void PostBeginContact(ButiEngine::Value_ptr< PhysicsObject > arg_p_self, ButiEngine::Value_ptr< PhysicsObject > arg_p_other);
    BUTIBULLET_API void PostEndContact(ButiEngine::Value_ptr< PhysicsObject > arg_p_self, ButiEngine::Value_ptr< PhysicsObject > arg_p_other);
    BUTIBULLET_API void ProcessContactCommands();


    BUTIBULLET_API void Initialize();
    BUTIBULLET_API virtual void OnDispose(const bool arg_explicitDisposing);

private:
    void UpdateObjectList();
    void AddObjectInternal(PhysicsObject* arg_p_obj);

    enum class ContactCommandType
    {
        Begin,
        End,
    };

    struct ContactCommand
    {
        ContactCommandType type;
        ButiEngine::Value_ptr<PhysicsObject> self;
        ButiEngine::Value_ptr<PhysicsObject> other;
    };

    btDefaultCollisionConfiguration* p_btCollisionConfig=nullptr;
    btCollisionDispatcher* p_btCollisionDispatcher = nullptr;
    btDbvtBroadphase* p_btBroadphase = nullptr;
    btSequentialImpulseConstraintSolver* p_btSolver = nullptr;
    btSoftRigidDynamicsWorld* p_btWorld = nullptr;
    btGhostPairCallback* p_btGhostPairCallback = nullptr;
    btSoftBodyWorldInfo* p_softBodyWorldInfo = nullptr;
    std::mutex mtx_sim;


    ButiEngine::List<ButiEngine::Value_ptr<PhysicsObject>> list_vlp_delayAddBodies;
    ButiEngine::List<ButiEngine::Value_ptr<Joint>> list_vlp_delayAddJoints;
    ButiEngine::List<ButiEngine::Value_ptr<PhysicsObject>> list_vlp_physicsObject;
    ButiEngine::List<ButiEngine::Value_ptr<Joint>> list_vlp_joint;

    std::vector<ContactCommand> vec_contactCommands;
};


class SpringJoint: public PhysicsObject  
{
public:
    BUTIBULLET_API static ButiEngine::Value_ptr<SpringJoint> Create();

    BUTIBULLET_API void SetBodyA(ButiEngine::Value_ptr<RigidBody> arg_p_body, const ButiEngine::Matrix4x4& arg_localJunctionPoint);
    BUTIBULLET_API void SetBodyB(ButiEngine::Value_ptr<RigidBody> arg_p_body, const ButiEngine::Matrix4x4& arg_localJunctionPoint);


    BUTIBULLET_API void SetLinearLowerLimit(const ButiEngine::Vector3& arg_linearLower);
    BUTIBULLET_API void SetLinearUpperLimit(const ButiEngine::Vector3& arg_linearUpper);
    BUTIBULLET_API void SetAngularLowerLimit(const ButiEngine::Vector3& arg_angularLower);
    BUTIBULLET_API void SetAngularUpperLimit(const ButiEngine::Vector3& arg_angularUpper);
    BUTIBULLET_API void SetLinearStiffness(const ButiEngine::Vector3& arg_value);
    BUTIBULLET_API void SetAngularStiffness(const ButiEngine::Vector3& arg_value);

    BUTIBULLET_API void OnDispose(const bool arg_explicitDisposing) override;
    BUTIBULLET_API void OnPrepareStepSimulation() override;
    BUTIBULLET_API void OnAfterStepSimulation() override;

    BUTIBULLET_API SpringJoint();
    BUTIBULLET_API void Initialize();
private:
    void RemoveFromBtWorld() override;

    btGeneric6DofSpringConstraint* p_btDofSpringConstraint;
    ButiEngine::Value_ptr<RigidBody> vlp_bodyA;
    ButiEngine::Value_ptr<RigidBody> vlp_bodyB;
    ButiEngine::Matrix4x4 localJunctionPointA;
    ButiEngine::Matrix4x4 localJunctionPointB;

    ButiEngine::Vector3 linearLowerLimit;
    ButiEngine::Vector3 linearUpperLimit;
    ButiEngine::Vector3 angularLowerLimit;
    ButiEngine::Vector3 angularUpperLimit;
    ButiEngine::Vector3 linearStiffness;
    ButiEngine::Vector3 angularStiffness;
};

}

#endif // !BUTIBULLET_PHYSICSWORLD_H
