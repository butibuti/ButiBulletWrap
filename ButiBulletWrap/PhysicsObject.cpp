#include"stdafx.h"
#include "PhysicsObject.h"
#include "PhysicsWorld.h"

ButiEngine::Value_ptr< ButiBullet::PhysicsWorld> ButiBullet::PhysicsObject::GetPhysicsWorld() const
{
    return vwp_ownerWorld.lock();
}

void ButiBullet::PhysicsObject::RemoveFromPhysicsWorld()
{
	auto world = vwp_ownerWorld.lock();
	if (world) {
		world->RemovePhysicsObject(value_from_this());
	}
}

void ButiBullet::PhysicsObject::OnCollisionEnter(PhysicsObject* arg_p_otherObject, ContactPoint* arg_p_contact)
{
	if (p_listener) {
		p_listener->OnCollisionEnter(arg_p_otherObject,arg_p_contact);
	}
}

void ButiBullet::PhysicsObject::OnCollisionLeave(PhysicsObject* arg_p_otherObject, ContactPoint* arg_p_contact)
{
	if (p_listener) {
		p_listener->OnCollisionLeave(arg_p_otherObject, arg_p_contact);
	}
}

void ButiBullet::PhysicsObject::OnCollisionStay(PhysicsObject* arg_p_otherObject, ContactPoint* arg_p_contact)
{
	if (p_listener) {
		p_listener->OnCollisionStay(arg_p_otherObject, arg_p_contact);
	}
}

void ButiBullet::PhysicsObject::OnPrepareStepSimulation()
{
	if (p_listener) {
		p_listener->OnBeforeStepSimulation_Deprecated();
	}
}

void ButiBullet::PhysicsObject::OnAfterStepSimulation()
{
	if (p_listener) {
		p_listener->OnAfterStepSimulation();
	}
}

ButiBullet::PhysicsObject::PhysicsObject(PhysicsObjectType arg_type):
 resourceType(arg_type)
, vwp_ownerWorld(nullptr)
, removing(false)
{
}

ButiBullet::PhysicsObject::~PhysicsObject()
{
	OnDispose(true);
}

void ButiBullet::PhysicsObject::Initialize()
{
}

void ButiBullet::PhysicsObject::OnDispose(const bool arg_explicitDisposing)
{
}

void ButiBullet::PhysicsObject::SetPhysicsWorld(ButiEngine::Value_ptr< PhysicsWorld> arg_vlp_owner)
{
    vwp_ownerWorld = arg_vlp_owner;
}

void ButiBullet::PhysicsObject::BeginContact(ButiEngine::Value_ptr< PhysicsObject> arg_vlp_otherObject)
{
	list_vlp_contactBodies.Add(arg_vlp_otherObject);
	OnCollisionEnter(arg_vlp_otherObject.get(), nullptr);
}

void ButiBullet::PhysicsObject::EndContact(ButiEngine::Value_ptr< PhysicsObject> arg_vlp_otherObject)
{
	if (!list_vlp_contactBodies.Remove(arg_vlp_otherObject)) return;
	OnCollisionLeave(arg_vlp_otherObject.get(), nullptr);
}
