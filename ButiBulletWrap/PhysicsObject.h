#ifndef BUTIBULLET_PHYSICSOBJECT_H
#define BUTIBULLET_PHYSICSOBJECT_H
#include"Common.h"
#include"ButiMemorySystem/ButiMemorySystem/MemoryAllocator.h"

namespace ButiBullet {
class PhysicsWorld;
class CollisionShape;

enum class PhysicsObjectType
{
    RigidBody,
    SoftBody,
    TriggerBody,
    Joint,
};

class PhysicsObject:public ButiEngine::enable_value_from_this<PhysicsObject>
{
public:

    ButiEngine::Value_ptr< PhysicsWorld >GetPhysicsWorld() const;

    BUTIBULLET_API void RemoveFromPhysicsWorld();

    const ButiEngine::List< ButiEngine::Value_ptr< PhysicsObject>>& ContactBodies() const { return list_vlp_contactBodies; }
    void SetEventListener(PhysicsDetail::IPhysicsObjectEventListener* arg_p_listener) { p_listener = arg_p_listener; }
    void SetOwnerData(void* arg_p_data) { p_ownerData =reinterpret_cast<PhysicsObject*> (arg_p_data); }
    void* OwnerData() const { return p_ownerData; }

    PhysicsObjectType GetPhysicsObjectType() const { return resourceType; }
protected:

    virtual void OnCollisionEnter(PhysicsObject* arg_p_otherObject, ContactPoint* arg_p_contact);
    virtual void OnCollisionLeave(PhysicsObject* arg_p_otherObject, ContactPoint* arg_p_contact);
    virtual void OnCollisionStay(PhysicsObject* arg_p_otherObject, ContactPoint* arg_p_contact);

    virtual void OnPrepareStepSimulation();
    virtual void OnAfterStepSimulation();


    PhysicsObject(PhysicsObjectType arg_type);
    virtual ~PhysicsObject();
    void Initialize(); 
    virtual void OnDispose(bool arg_explicitDisposing);

private:
    virtual void RemoveFromBtWorld() = 0;

    void SetPhysicsWorld(ButiEngine::Value_ptr< PhysicsWorld > arg_vlp_ownerWorld);
    void BeginContact(ButiEngine::Value_ptr< PhysicsObject> arg_p_otherObject);
    void EndContact(ButiEngine::Value_ptr< PhysicsObject>  arg_p_otherObject);

    PhysicsObjectType resourceType;
    ButiEngine::Value_weak_ptr< PhysicsWorld > vwp_ownerWorld;
    bool removing;

    ButiEngine::List<ButiEngine::Value_ptr< PhysicsObject>> list_vlp_contactBodies;
    PhysicsDetail::IPhysicsObjectEventListener* p_listener = nullptr;
    void* p_ownerData = nullptr;

    friend class PhysicsWorld;
};
}

#endif // !BUTIBULLET_PHYSICSOBJECT_H
