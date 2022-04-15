#include"stdafx.h"
#include "PhysicsWorld.h"
#include "Joint.h"

void ButiBullet::Joint::RemoveFromPhysicsWorld()
{
    if (!vlp_world) return;
    vlp_world->RemoveJoint(value_from_this());
}

ButiBullet::Joint::Joint():vlp_world(nullptr),removing(false) {

}
