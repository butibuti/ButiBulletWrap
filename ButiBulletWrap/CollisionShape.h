#ifndef BUTIBULLET_COLLISIONSHAPE_H
#define BUTIBULLET_COLLISIONSHAPE_H
#include"Common.h"
#include"ButiMemorySystem/ButiMemorySystem/ButiList.h"
namespace ButiEngine {
namespace ButiRendering {
class MeshPrimitiveBase;
}

}
namespace ButiBullet {

class PhysicsWorld;
class PhysicsObject;
class Joint;
class ContactPoint;
extern class CollisionShape
{
public:

	BUTIBULLET_API CollisionShape();
	BUTIBULLET_API virtual ~CollisionShape();
	void SetTrigger(const bool arg_enabled) { isTrigger = arg_enabled; }
	bool IsTrigger() const { return isTrigger; }
	inline void SetPosition(const ButiEngine::Vector3& arg_pos) { position = arg_pos; }
	inline const ButiEngine::Vector3& GetPosition() const { return position; }
	inline void SetRotation(const ButiEngine::Quat & arg_rotation) { rotation = arg_rotation; }
	inline const ButiEngine::Quat& GetRotation() const { return rotation; }


	BUTIBULLET_API bool Initialize();
	BUTIBULLET_API bool Initialize(btCollisionShape* arg_shape);

	btCollisionShape* GetBtCollisionShape() const { return p_shape; }

private:
	btCollisionShape* p_shape;
	ButiEngine::Vector3 position;
	ButiEngine::Quat rotation;
	bool isTrigger;
};

extern class PlaneCollisionShape: public CollisionShape
{
public:

	BUTIBULLET_API static ButiEngine::Value_ptr<PlaneCollisionShape> Create(const ButiEngine::Vector3& arg_direction);

	BUTIBULLET_API PlaneCollisionShape();
	BUTIBULLET_API virtual ~PlaneCollisionShape();

	BUTIBULLET_API bool Initialize();
	BUTIBULLET_API bool Initialize(const ButiEngine::Vector3& arg_direction);
};
extern class BoxCollisionShape: public CollisionShape
{
public:

	BUTIBULLET_API static ButiEngine::Value_ptr<BoxCollisionShape> Create(const ButiEngine::Vector3& arg_size);
	BUTIBULLET_API static ButiEngine::Value_ptr<BoxCollisionShape> Create(const float arg_x, const float arg_y, const float arg_z);

	BUTIBULLET_API BoxCollisionShape();
	BUTIBULLET_API virtual ~BoxCollisionShape();

	BUTIBULLET_API bool Initialize();
	BUTIBULLET_API bool Initialize(const ButiEngine::Vector3& arg_size);
	BUTIBULLET_API bool Initialize(const float arg_x, const float arg_y, const float arg_z);
};

extern class CylinderShape : public CollisionShape {
public:
	BUTIBULLET_API static ButiEngine::Value_ptr<CylinderShape> Create(const ButiEngine::Vector3& arg_scale);

	BUTIBULLET_API CylinderShape();
	BUTIBULLET_API virtual ~CylinderShape();

	BUTIBULLET_API bool Initialize();
	BUTIBULLET_API bool Initialize(const ButiEngine::Vector3& arg_scale);
};

extern class ConeShape : public CollisionShape {
public:
	BUTIBULLET_API static ButiEngine::Value_ptr<ConeShape> Create(const float arg_radius, const float arg_height);

	BUTIBULLET_API ConeShape();
	BUTIBULLET_API virtual ~ConeShape();

	BUTIBULLET_API bool Initialize();
	BUTIBULLET_API bool Initialize(const float arg_radius, const float arg_height);
};

extern class SphereCollisionShape
	: public CollisionShape
{
public:

	BUTIBULLET_API static ButiEngine::Value_ptr<SphereCollisionShape> Create(const float arg_radius);

	BUTIBULLET_API SphereCollisionShape();
	BUTIBULLET_API virtual ~SphereCollisionShape();

	BUTIBULLET_API bool Initialize();
	BUTIBULLET_API bool Initialize(const float arg_radius);
};

class CapsuleCollisionShape	: public CollisionShape
{
public:

	BUTIBULLET_API static ButiEngine::Value_ptr<CapsuleCollisionShape> Create(const float arg_radius, const float arg_height);

	BUTIBULLET_API CapsuleCollisionShape();
	BUTIBULLET_API virtual ~CapsuleCollisionShape();

	BUTIBULLET_API bool Initialize();
	BUTIBULLET_API bool Initialize(const float arg_radius, const float arg_height);
};

extern class MeshCollisionShape	: public CollisionShape
{
public:

	BUTIBULLET_API static ButiEngine::Value_ptr<MeshCollisionShape> Create(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh);
	BUTIBULLET_API static ButiEngine::Value_ptr<MeshCollisionShape> Create(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh, const ButiEngine::Matrix4x4& arg_transform);

	BUTIBULLET_API MeshCollisionShape();
	BUTIBULLET_API virtual ~MeshCollisionShape();

	BUTIBULLET_API bool Initialize();
	BUTIBULLET_API bool Initialize(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh);
	BUTIBULLET_API bool Initialize(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh, const ButiEngine::Matrix4x4& arg_transform);

private:
	BUTIBULLET_API bool InitInternal(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh, const ButiEngine::Matrix4x4* arg_transform);

	btTriangleIndexVertexArray* p_btMeshData=nullptr;
};


namespace PhysicsDetail {

class BtShapeManager
{
public:
	BtShapeManager();
	~BtShapeManager();
	void AddShape(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape, const ButiEngine::Matrix4x4& arg_vlp_localTransform);
	inline void AddShape(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape) {
		static ButiEngine::Matrix4x4 indentity;
		AddShape(arg_vlp_shape, indentity);
	}
	bool IsEmpty() const;
	btCollisionShape* GetBtCollisionShape();
private:


	void Refresh();


	ButiEngine::List<ButiEngine::Value_ptr<CollisionShape>> list_vlp_collisionShapes;
	btCompoundShape* p_btCompoundShape;
	btCollisionShape* p_activeShape;
	bool dirty;
};

}

}

#endif // !BUTIBULLET_COLLISIONSHAPE_H
