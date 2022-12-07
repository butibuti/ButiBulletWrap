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


class PhysicsWorld:public ButiEngine::enable_value_from_this<PhysicsWorld>,public IPhysicsWorld
{
public:
    BUTIBULLET_API PhysicsWorld(const std::int32_t arg_iteration=1);
    BUTIBULLET_API virtual ~PhysicsWorld();

    BUTIBULLET_API void AddPhysicsObject(ButiEngine::Value_ptr< PhysicsObject >arg_vlp_physicsObject)override;
    BUTIBULLET_API void AddJoint(ButiEngine::Value_ptr< IJoint> arg_vlp_joint)override;

    BUTIBULLET_API void RemovePhysicsObject(ButiEngine::Value_ptr< PhysicsObject > arg_vlp_physicsObject)override;
    BUTIBULLET_API void RemoveJoint(ButiEngine::Value_ptr< IJoint> arg_vlp_joint)override;

	BUTIBULLET_API bool Raycast(const ButiEngine::Vector3& arg_origin, const ButiEngine::Vector3& arg_direction, const float arg_maxDistance, const bool arg_queryTrigger, PhysicsRaycastResult* arg_p_outResult = nullptr)override;
	BUTIBULLET_API bool Raycast(const ButiEngine::Vector3& arg_origin, const ButiEngine::Vector3& arg_direction, const float arg_maxDistance, PhysicsRaycastResult* arg_p_outResult = nullptr) override{ return Raycast(arg_origin, arg_direction, arg_maxDistance, false, arg_p_outResult); }

	BUTIBULLET_API bool RaycastAllHit(const ButiEngine::Vector3& arg_origin, const ButiEngine::Vector3& arg_direction, const float arg_maxDistance, const std::uint32_t arg_groupMask, const bool arg_queryTrigger, ButiEngine::List<PhysicsRaycastResult>* arg_p_outResult = nullptr)override;
	BUTIBULLET_API bool RaycastAllHit(const ButiEngine::Vector3& arg_origin, const ButiEngine::Vector3& arg_direction, const float arg_maxDistance, const std::uint32_t arg_groupMask, ButiEngine::List<PhysicsRaycastResult>* arg_p_outResult = nullptr)override { return RaycastAllHit(arg_origin, arg_direction, arg_maxDistance, arg_groupMask, false, arg_p_outResult); }


    btSoftRigidDynamicsWorld* GetBtWorld() { return m_p_btWorld; }
    btSoftBodyWorldInfo* GetSoftBodyWorldInfo() const { return m_p_softBodyWorldInfo; }
    BUTIBULLET_API void StepSimulation(const float arg_elapsedSeconds);
    BUTIBULLET_API void RenderDebug(RenderingContext* arg_p_context);

    BUTIBULLET_API void PostBeginContact(ButiEngine::Value_ptr< PhysicsObject > arg_p_self, ButiEngine::Value_ptr< PhysicsObject > arg_p_other);
    BUTIBULLET_API void PostEndContact(ButiEngine::Value_ptr< PhysicsObject > arg_p_self, ButiEngine::Value_ptr< PhysicsObject > arg_p_other);
    BUTIBULLET_API void ProcessContactCommands();


    BUTIBULLET_API void Initialize()override;
    BUTIBULLET_API void OnDispose(const bool arg_explicitDisposing)override;
	BUTIBULLET_API void SetIsPause(const bool arg_isPause) override;
	BUTIBULLET_API bool GetIsPause()override;

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

    btDefaultCollisionConfiguration* m_p_btCollisionConfig=nullptr;
    btCollisionDispatcher* m_p_btCollisionDispatcher = nullptr;
    btDbvtBroadphase* m_p_btBroadphase = nullptr;
    btSequentialImpulseConstraintSolver* m_p_btSolver = nullptr;
    btSoftRigidDynamicsWorld* m_p_btWorld = nullptr;
    btGhostPairCallback* m_p_btGhostPairCallback = nullptr;
    btSoftBodyWorldInfo* m_p_softBodyWorldInfo = nullptr;
    std::mutex m_mtx_sim;


    ButiEngine::List<ButiEngine::Value_ptr<PhysicsObject>> m_list_vlp_delayAddBodies;
    ButiEngine::List<ButiEngine::Value_ptr<IJoint>> m_list_vlp_delayAddJoints;
    ButiEngine::List<ButiEngine::Value_ptr<PhysicsObject>> m_list_vlp_physicsObject;
    ButiEngine::List<ButiEngine::Value_ptr<IJoint>> m_list_vlp_joint;

    ButiEngine::List<ContactCommand> m_list_contactCommands;

    std::int32_t m_iteration;
    float m_iterationTimeUnit;
	bool m_isPause=false;
};



}

#endif // !BUTIBULLET_PHYSICSWORLD_H
