#ifndef _BUITBULLET_MOTIONSTATE_H
#define _BUITBULLET_MOTIONSTATE_H
#include"RigidBody.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "LinearMath/btMotionState.h"
namespace ButiBullet{

namespace PhysicsDetail {

class SynchronizeMotionState
    : public btDefaultMotionState
{
public:
    PhysicsObject* p_owner;

    SynchronizeMotionState(PhysicsObject* arg_p_owner, const btTransform& startTrans = btTransform::getIdentity(), const btTransform& centerOfMassOffset = btTransform::getIdentity())
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
}

#endif // !_BUITBULLET_MOTIONSTATE_H

