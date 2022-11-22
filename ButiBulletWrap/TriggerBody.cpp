#include"stdafx.h"
#include<assert.h>
#pragma warning(disable: 5033)
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#pragma warning(default : 5033)
#include "PhysicsWorld.h"
#include "PhysicsManager.h"
#include"MotionState.h"
#include "BulletUtil.h"

#include "TriggerBody.h"

ButiEngine::Value_ptr<ButiBullet::TriggerBody> ButiBullet::TriggerBody::Create(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape)
{
	auto output = ButiEngine::make_value<TriggerBody>();
	output->Initialize(arg_vlp_shape);
	return output;
}

void ButiBullet::TriggerBody::AddCollisionShape(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape)
{
	shapeManager.AddShape(arg_vlp_shape);
	dirtyFlags |= DirtyFlags_Shapes;
}

void ButiBullet::TriggerBody::SetTransform(const ButiEngine::Matrix4x4& arg_transform)
{
	transform = arg_transform;
	dirtyFlags |= DirtyFlags_Transform;
}

void ButiBullet::TriggerBody::SetCollisionGroup(const std::uint32_t arg_value)
{
	if (group != arg_value) {
		group = arg_value;
		dirtyFlags |= DirtyFlags_Group;
	}
}

void ButiBullet::TriggerBody::SetCollisionGroupMask(const std::uint32_t arg_value)
{
	if (groupMask != arg_value) {
		groupMask = arg_value;
		dirtyFlags |= DirtyFlags_Group;
	}
}

void ButiBullet::TriggerBody::OnDispose(const bool arg_explicitDisposing)
{
	DeleteBtObject();
	PhysicsObject::OnDispose(arg_explicitDisposing);
}

void ButiBullet::TriggerBody::OnPrepareStepSimulation()
{
	PhysicsObject::OnPrepareStepSimulation();

	if (!p_btRigidBody || (dirtyFlags & (DirtyFlags_InitialUpdate))) {
		CreateBtObject();
		dirtyFlags &= ~DirtyFlags_Shapes;	
		dirtyFlags &= ~DirtyFlags_Group;	
		dirtyFlags &= ~DirtyFlags_Transform;
	}

	if (dirtyFlags & DirtyFlags_Shapes) {
		GetPhysicsWorld()->GetBtWorld()->removeCollisionObject(p_btRigidBody);
		p_btRigidBody->setCollisionShape(shapeManager.GetBtCollisionShape());
		GetPhysicsWorld()->GetBtWorld()->addCollisionObject(p_btRigidBody, group, groupMask);
		dirtyFlags &= ~DirtyFlags_Group;
	}

	if (dirtyFlags & DirtyFlags_Group) {
		ReaddToWorld();
	}

	if (dirtyFlags & DirtyFlags_Transform) {
		btTransform btTransform;
		btTransform.setFromOpenGLMatrix(reinterpret_cast<btScalar*>( &transform));
		p_btRigidBody->setWorldTransform(btTransform);
		p_btRigidBody->getMotionState()->setWorldTransform(btTransform);
		p_btRigidBody->activate();
	}

	dirtyFlags = DirtyFlags_None;
}

void ButiBullet::TriggerBody::OnAfterStepSimulation()
{
	PhysicsObject::OnAfterStepSimulation();
}

ButiBullet::TriggerBody::TriggerBody()
	: PhysicsObject(PhysicsObjectType::TriggerBody),p_btRigidBody(nullptr),btWorldAdded(false)
	, shapeManager()
	, transform()
{
}
ButiBullet::TriggerBody::~TriggerBody()
{
	if (p_btRigidBody != nullptr)
	{
		btMotionState* state = p_btRigidBody->getMotionState();
		delete (state);
		delete (p_btRigidBody);
		p_btRigidBody = nullptr;
	}
}
void ButiBullet::TriggerBody::Initialize()
{
}

void ButiBullet::TriggerBody::Initialize(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape)
{
	AddCollisionShape(arg_vlp_shape);
}

void ButiBullet::TriggerBody::RemoveFromBtWorld()
{
	if (btWorldAdded) {
		GetPhysicsWorld()->GetBtWorld()->removeCollisionObject(p_btRigidBody);
		btWorldAdded = false;

		if (p_btRigidBody) {
			delete p_btRigidBody;
			p_btRigidBody = nullptr;
		}
	}
}

void ButiBullet::TriggerBody::Activate()
{
	dirtyFlags |= DirtyFlags_All;
}

void ButiBullet::TriggerBody::CreateBtObject()
{
	DeleteBtObject();

	assert(!shapeManager.IsEmpty());
	btTransform initialTransform;
	{
		initialTransform.setFromOpenGLMatrix(reinterpret_cast<const btScalar*>(&transform));
		initialTransform.getOrigin().setX(initialTransform.getOrigin().x());
		initialTransform.getOrigin().setY(initialTransform.getOrigin().y());
		initialTransform.getOrigin().setZ(initialTransform.getOrigin().z());
	}
	btMotionState* motionState = nullptr;
	motionState= new PhysicsDetail::SynchronizeMotionState(this, initialTransform);



	btRigidBody::btRigidBodyConstructionInfo bodyInfo(0, motionState, shapeManager.GetBtCollisionShape(), btVector3(0.0f, 0.0f, 0.0f));
	p_btRigidBody = new btRigidBody(bodyInfo);
	p_btRigidBody->setUserPointer(weakAddress());

	p_btRigidBody->setCollisionShape(shapeManager.GetBtCollisionShape());
	p_btRigidBody->setCollisionFlags( btCollisionObject::CF_NO_CONTACT_RESPONSE);

	btTransform btTransform;
	btTransform.setFromOpenGLMatrix(reinterpret_cast<btScalar*>(&transform));
	p_btRigidBody->setWorldTransform(btTransform);

	GetPhysicsWorld()->GetBtWorld()->addCollisionObject(p_btRigidBody, group, groupMask);
	btWorldAdded = true;
}

void ButiBullet::TriggerBody::DeleteBtObject()
{
	if (p_btRigidBody) {
		delete p_btRigidBody;
		p_btRigidBody = nullptr;
	}
}

void ButiBullet::TriggerBody::ReaddToWorld()
{
	if (p_btRigidBody) {
		GetPhysicsWorld()->GetBtWorld()->removeCollisionObject(p_btRigidBody);
		GetPhysicsWorld()->GetBtWorld()->addCollisionObject(p_btRigidBody, group, groupMask);
	}
}
