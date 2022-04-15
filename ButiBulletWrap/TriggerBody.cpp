#include"stdafx.h"
#include<assert.h>
#pragma warning(disable: 5033)	// disable warning in bullet headers
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#pragma warning(default : 5033)
#include "PhysicsWorld.h"
#include "PhysicsManager.h"
#include "BulletUtil.h"

#include "TriggerBody.h"


class ButiBullet::TriggerBody::LocalGhostObject : public btGhostObject
{
public:
	ButiEngine::Value_ptr<TriggerBody> vlp_owner;

	LocalGhostObject(ButiEngine::Value_ptr<TriggerBody> arg_vlp_owner)
		: vlp_owner(vlp_owner)
	{}

	virtual void addOverlappingObjectInternal(btBroadphaseProxy* otherProxy, btBroadphaseProxy* thisProxy = 0) override
	{
		btCollisionObject* otherObject = (btCollisionObject*)otherProxy->m_clientObject;
		assert(otherObject);
		std::int32_t index = m_overlappingObjects.findLinearSearch(otherObject);
		if (index == m_overlappingObjects.size())
		{
			m_overlappingObjects.push_back(otherObject);

			auto bodyB = reinterpret_cast<ButiEngine::Value_weak_ptr<PhysicsObject>*>(otherObject->getUserPointer())->lock();

			auto world = vlp_owner->GetPhysicsWorld();
			world->PostBeginContact(vlp_owner, bodyB);

			// RigidBody 自体は衝突検知機能を持たないので、TiggerBody 側から通知を送る
			if (bodyB->GetPhysicsObjectType() == PhysicsObjectType::RigidBody) {
				world->PostBeginContact(bodyB, vlp_owner);
			}
		}
	}

	virtual void removeOverlappingObjectInternal(btBroadphaseProxy* otherProxy, btDispatcher* dispatcher, btBroadphaseProxy* thisProxy = 0) override
	{
		btCollisionObject* otherObject = (btCollisionObject*)otherProxy->m_clientObject;
		assert(otherObject);
		std::int32_t index = m_overlappingObjects.findLinearSearch(otherObject);
		if (index < m_overlappingObjects.size())
		{
			m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size() - 1];
			m_overlappingObjects.pop_back();

			auto bodyB = reinterpret_cast<ButiEngine::Value_weak_ptr<PhysicsObject>*>(otherObject->getUserPointer())->lock();

			auto world = vlp_owner->GetPhysicsWorld();
			world->PostEndContact(vlp_owner, bodyB);

			// RigidBody 自体は衝突検知機能を持たないので、TiggerBody 側から通知を送る
			if (bodyB->GetPhysicsObjectType() == PhysicsObjectType::RigidBody) {
				world->PostEndContact(bodyB, vlp_owner);
			}
		}
	}
};


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

	if (!p_btGhostObject || (dirtyFlags & (DirtyFlags_InitialUpdate))) {
		CreateBtObject();
		dirtyFlags &= ~DirtyFlags_Shapes;		// createBtObject() の中でまとめて処理されるため、↓で処理する必要はない
		dirtyFlags &= ~DirtyFlags_Group;		// createBtObject() の中でまとめて処理されるため、↓で処理する必要はない
		dirtyFlags &= ~DirtyFlags_Transform;	// createBtObject() の中でまとめて処理されるため、↓で処理する必要はない
	}

	if (dirtyFlags & DirtyFlags_Shapes) {
		p_btGhostObject->setCollisionShape(shapeManager.GetBtCollisionShape());
		ReaddToWorld();
		dirtyFlags &= ~DirtyFlags_Group;	// readdToWorld() 処理済み
	}

	if (dirtyFlags & DirtyFlags_Group) {
		ReaddToWorld();
	}

	if (dirtyFlags & DirtyFlags_Transform) {
		btTransform btTransform;
		btTransform.setFromOpenGLMatrix(reinterpret_cast<btScalar*>( &transform));
		p_btGhostObject->setWorldTransform(btTransform);
	}

	dirtyFlags = DirtyFlags_None;
}

void ButiBullet::TriggerBody::OnAfterStepSimulation()
{
	PhysicsObject::OnAfterStepSimulation();
}

ButiBullet::TriggerBody::TriggerBody()
	: PhysicsObject(PhysicsObjectType::TriggerBody)
{
}

void ButiBullet::TriggerBody::Initialize()
{
	PhysicsObject::Initialize();
}

void ButiBullet::TriggerBody::Initialize(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape)
{
	Initialize();
	AddCollisionShape(arg_vlp_shape);
}

void ButiBullet::TriggerBody::RemoveFromBtWorld()
{
	if (btWorldAdded) {
		GetPhysicsWorld()->GetBtWorld()->removeCollisionObject(p_btGhostObject);
		btWorldAdded = false;

		if (p_btGhostObject) {
			delete p_btGhostObject;
			p_btGhostObject = nullptr;
		}
	}
}

void ButiBullet::TriggerBody::CreateBtObject()
{
	DeleteBtObject();

	assert(!shapeManager.IsEmpty());

	p_btGhostObject = new LocalGhostObject(ButiEngine::dynamic_value_ptr_cast<TriggerBody>(value_from_this()) );
	p_btGhostObject->setUserPointer(weakAddress());

	// setCollisionShape() は World に追加する前に必須
	p_btGhostObject->setCollisionShape(shapeManager.GetBtCollisionShape());

	// btCollisionObject::CF_NO_CONTACT_RESPONSE が付加されると、
	// 他のオブジェクトと物理シミュレーションで接触しないことを示す (すり抜ける)
	p_btGhostObject->setCollisionFlags(p_btGhostObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

	//m_btGhostObject->setWorldTransform(detail::BulletUtil::LNMatrixToBtTransform(mtmp));

	// addCollisionObject() した瞬間に周囲のオブジェクトと衝突判定が行われるため、初期姿勢を addCollisionObject() の前に設定しておく必要がある。
	btTransform btTransform;
	btTransform.setFromOpenGLMatrix(reinterpret_cast<btScalar*>(&transform));
	p_btGhostObject->setWorldTransform(btTransform);

	GetPhysicsWorld()->GetBtWorld()->addCollisionObject(p_btGhostObject, group, groupMask);
	btWorldAdded = true;
}

void ButiBullet::TriggerBody::DeleteBtObject()
{
	if (p_btGhostObject) {
		delete p_btGhostObject;
		p_btGhostObject = nullptr;
	}
}

void ButiBullet::TriggerBody::ReaddToWorld()
{
	if (p_btGhostObject) {
		GetPhysicsWorld()->GetBtWorld()->removeCollisionObject(p_btGhostObject);
		GetPhysicsWorld()->GetBtWorld()->addCollisionObject(p_btGhostObject, group, groupMask);
	}
}
