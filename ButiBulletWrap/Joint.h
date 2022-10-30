#ifndef BUTIBULLET_JOINT_H
#define BUTIBULLET_JOINT_H
#include"Common.h"
namespace ButiBullet {
class Joint_P2P :public ButiEngine::enable_value_from_this<Joint_P2P>,public IJoint
{
public:

    ButiEngine::Value_ptr< PhysicsWorld> GetPhysicsWorld() const override{ return m_vlp_world; }
    bool GetIsRemoving() const override { return m_removing; }

    void RemoveFromBtWorld() override;
    void RemoveFromPhysicsWorld();

    Joint_P2P(ButiEngine::Value_ptr< IRigidBody> arg_vlp_object, const ButiEngine::Vector3& arg_jointPosition,
        const float arg_durability);
    Joint_P2P(ButiEngine::Value_ptr< IRigidBody> arg_vlp_objectA, const ButiEngine::Vector3& arg_jointPositionA,
        ButiEngine::Value_ptr< IRigidBody> arg_vlp_objectB, const ButiEngine::Vector3& arg_jointPositionB,
        const float arg_durability);
    ~Joint_P2P();
    void SetIsRemoving(const bool arg_isRemove)override { m_removing = arg_isRemove; }
    void SetPhysicsWorld(ButiEngine::Value_ptr<PhysicsWorld> arg_world) { m_vlp_world = arg_world; }
private:

    btTypedConstraint* m_p_btConstraint;
    ButiEngine::Value_ptr< PhysicsWorld> m_vlp_world;
    bool m_removing;

    friend class PhysicsWorld;
};


class Joint_spring :public ButiEngine::enable_value_from_this<Joint_spring>, public IJoint
{
public:

    ButiEngine::Value_ptr< PhysicsWorld> GetPhysicsWorld() const override { return m_vlp_world; }
    bool GetIsRemoving() const override { return m_removing; }
    void SetIsRemoving(const bool arg_isRemove)override { m_removing = arg_isRemove; }
    void SetPhysicsWorld(ButiEngine::Value_ptr<PhysicsWorld> arg_world) { m_vlp_world = arg_world; }
    BUTIBULLET_API void SetBodyA(ButiEngine::Value_ptr<IRigidBody> arg_vlp_body, const ButiEngine::Matrix4x4& arg_localJunctionPoint);
    BUTIBULLET_API void SetBodyB(ButiEngine::Value_ptr<IRigidBody> arg_vlp_body, const ButiEngine::Matrix4x4& arg_localJunctionPoint);


    BUTIBULLET_API void SetLinearLowerLimit(const ButiEngine::Vector3& arg_linearLower);
    BUTIBULLET_API void SetLinearUpperLimit(const ButiEngine::Vector3& arg_linearUpper);
    BUTIBULLET_API void SetAngularLowerLimit(const ButiEngine::Vector3& arg_angularLower);
    BUTIBULLET_API void SetAngularUpperLimit(const ButiEngine::Vector3& arg_angularUpper);
    BUTIBULLET_API void SetLinearStiffness(const ButiEngine::Vector3& arg_value);
    BUTIBULLET_API void SetAngularStiffness(const ButiEngine::Vector3& arg_value);
    BUTIBULLET_API void SetDurability(const float arg_durability);

    BUTIBULLET_API void OnPrepareStepSimulation() override;

    BUTIBULLET_API Joint_spring();
    BUTIBULLET_API ~Joint_spring();
    BUTIBULLET_API Joint_spring(ButiEngine::Value_ptr<IRigidBody> arg_vlp_bodyA, const ButiEngine::Matrix4x4& arg_localJunctionPointA
    , ButiEngine::Value_ptr<IRigidBody> arg_vlp_bodyB, const ButiEngine::Matrix4x4& arg_localJunctionPointB,const float arg_durability);


    void RemoveFromBtWorld() override;

private:

    ButiEngine::Value_ptr< PhysicsWorld> m_vlp_world;
    bool m_removing, m_isParameterChanged;
    btGeneric6DofSpringConstraint* m_p_btDofSpringConstraint;
    ButiEngine::Value_ptr<IRigidBody> m_vlp_bodyA;
    ButiEngine::Value_ptr<IRigidBody> m_vlp_bodyB;
    ButiEngine::Matrix4x4 m_localJunctionPointA;
    ButiEngine::Matrix4x4 m_localJunctionPointB;

    ButiEngine::Vector3 m_linearLowerLimit;
    ButiEngine::Vector3 m_linearUpperLimit;
    ButiEngine::Vector3 m_angularLowerLimit;
    ButiEngine::Vector3 m_angularUpperLimit;
    ButiEngine::Vector3 m_linearStiffness;
    ButiEngine::Vector3 m_angularStiffness;
    float m_durability = 0.0f;
};

}

#endif // !BUTIBULLET_JOINT_H
