#include"stdafx.h"

#include "CollisionShape.h"

#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#include"BulletUtil.h"
#include"ButiRendering_Dx12/Header/MeshPrimitive.h"

ButiBullet::CollisionShape::CollisionShape():p_shape(nullptr), position(), isTrigger(false)
{
}

ButiBullet::CollisionShape::~CollisionShape()
{
    delete p_shape;
}

bool ButiBullet::CollisionShape::Initialize()
{
    return true;
}

bool ButiBullet::CollisionShape::Initialize(btCollisionShape* arg_p_shape)
{
    return p_shape= arg_p_shape;
}

ButiEngine::Value_ptr<ButiBullet::BoxCollisionShape> ButiBullet::BoxCollisionShape::Create(const ButiEngine::Vector3& arg_size)
{
    auto output = ButiEngine::make_value<ButiBullet::BoxCollisionShape>();
    output->Initialize(arg_size);
    return output;
}

ButiEngine::Value_ptr<ButiBullet::BoxCollisionShape> ButiBullet::BoxCollisionShape::Create(const float arg_x, const float arg_y, const float arg_z)
{
    return Create(ButiEngine:: Vector3(arg_x, arg_y, arg_z));
}

ButiBullet::BoxCollisionShape::BoxCollisionShape()
{
}

ButiBullet::BoxCollisionShape::~BoxCollisionShape()
{
}

bool ButiBullet::BoxCollisionShape::Initialize()
{
    return CollisionShape::Initialize();
}

bool ButiBullet::BoxCollisionShape::Initialize(const ButiEngine::Vector3& arg_size)
{
    return CollisionShape::Initialize(new btBoxShape(PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_size * 0.5f)));
}

bool ButiBullet::BoxCollisionShape::Initialize(const float arg_x, const float arg_y, const float arg_z)
{
    return Initialize(ButiEngine::Vector3(arg_x, arg_y,arg_z));
}

ButiEngine::Value_ptr<ButiBullet::SphereCollisionShape> ButiBullet::SphereCollisionShape::Create(const float arg_radius)
{
    auto output = ButiEngine::make_value<SphereCollisionShape>();
    output->Initialize(arg_radius);
    return output;
}

ButiBullet::SphereCollisionShape::SphereCollisionShape()
{
}

ButiBullet::SphereCollisionShape::~SphereCollisionShape()
{
}

bool ButiBullet::SphereCollisionShape::Initialize()
{
    return CollisionShape::Initialize();
}

bool ButiBullet::SphereCollisionShape::Initialize(const float arg_radius)
{
    return CollisionShape::Initialize(new btSphereShape(arg_radius));
}

ButiEngine::Value_ptr<ButiBullet::CapsuleCollisionShape> ButiBullet::CapsuleCollisionShape::Create(const float arg_radius, const float arg_height)
{
    auto output = ButiEngine::make_value<CapsuleCollisionShape>();
    output->Initialize(arg_radius, arg_height);
    return output;
}

ButiBullet::CapsuleCollisionShape::CapsuleCollisionShape()
{
}

ButiBullet::CapsuleCollisionShape::~CapsuleCollisionShape()
{
}

bool ButiBullet::CapsuleCollisionShape::Initialize()
{
    return CollisionShape::Initialize();
}

bool ButiBullet::CapsuleCollisionShape::Initialize(const float arg_radius, const float arg_height)
{
    return CollisionShape::Initialize(new btCapsuleShape(arg_radius, arg_height - (arg_radius * 2.0f)));
}

ButiEngine::Value_ptr<ButiBullet::MeshCollisionShape> ButiBullet::MeshCollisionShape::Create(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh )
{
    auto output = ButiEngine::make_value<ButiBullet::MeshCollisionShape>();
    output->Initialize(arg_p_mesh);
    return output;
}

ButiEngine::Value_ptr<ButiBullet::MeshCollisionShape> ButiBullet::MeshCollisionShape::Create(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh, const ButiEngine::Matrix4x4& arg_transform)
{
    auto output = ButiEngine::make_value<ButiBullet::MeshCollisionShape>();
    output->Initialize(arg_p_mesh);
    return output;
}

ButiBullet::MeshCollisionShape::MeshCollisionShape()
{
}

ButiBullet::MeshCollisionShape::~MeshCollisionShape()
{
}

bool ButiBullet::MeshCollisionShape::Initialize()
{
    return CollisionShape::Initialize();
}

bool ButiBullet::MeshCollisionShape::Initialize(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh)
{
    return InitInternal(arg_p_mesh,nullptr);
}

bool ButiBullet::MeshCollisionShape::Initialize(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh, const ButiEngine::Matrix4x4& arg_transform)
{
    return InitInternal(arg_p_mesh, &arg_transform);
}

bool ButiBullet::MeshCollisionShape::InitInternal(const ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh, const ButiEngine::Matrix4x4* arg_transform)
{
	if (!arg_p_mesh) return false;
	if (p_btMeshData) return false;




	btIndexedMesh btMesh;
	btMesh.m_numTriangles = arg_p_mesh->GetIndexCount() / 3;
	btMesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(arg_p_mesh->GetIndexData());
	btMesh.m_triangleIndexStride = sizeof(std::uint32_t) * 3;
	btMesh.m_numVertices = arg_p_mesh->GetVertexCount();
	btMesh.m_vertexBase = reinterpret_cast<const unsigned char*>(arg_p_mesh->GetVertexData());
	btMesh.m_vertexStride = sizeof(ButiEngine::Vertex::Vertex_UV_Normal);

	PHY_ScalarType indexFormat = PHY_INTEGER;

	p_btMeshData = new btTriangleIndexVertexArray();
	p_btMeshData->addIndexedMesh(btMesh, indexFormat);

	auto output = CollisionShape::Initialize(new btBvhTriangleMeshShape(p_btMeshData, true));
	return output;


}

ButiBullet::PhysicsDetail::BtShapeManager::BtShapeManager()
{
}

ButiBullet::PhysicsDetail::BtShapeManager::~BtShapeManager()
{
}

void ButiBullet::PhysicsDetail::BtShapeManager::AddShape(ButiEngine::Value_ptr<CollisionShape> arg_vlp_shape, const ButiEngine::Matrix4x4& localTransform)
{
	if (!arg_vlp_shape) return;

	list_vlp_collisionShapes.Add(arg_vlp_shape);


	dirty = true;
}

bool ButiBullet::PhysicsDetail::BtShapeManager::IsEmpty() const
{
	return !list_vlp_collisionShapes.GetSize();
}

btCollisionShape* ButiBullet::PhysicsDetail::BtShapeManager::GetBtCollisionShape()
{
	if (dirty)
	{
		Refresh();
		dirty = false;
	}
	return p_activeShape;
}

void ButiBullet::PhysicsDetail::BtShapeManager::Refresh()
{
	static ButiEngine::Quat IdentityQuat;
	btCollisionShape* shape;
	if (list_vlp_collisionShapes.GetSize() == 1 &&
		(*list_vlp_collisionShapes.begin())->GetPosition() == ButiEngine::Vector3Const::Zero &&
		(*list_vlp_collisionShapes.begin())->GetRotation() == IdentityQuat) {
		shape = (*list_vlp_collisionShapes.begin())->GetBtCollisionShape();
	}
	else
	{
		if (p_btCompoundShape == nullptr)
		{
			p_btCompoundShape = new btCompoundShape();
		}
		else
		{
			for (std::int32_t i = p_btCompoundShape->getNumChildShapes() - 1; i >= 0; i--)
			{
				p_btCompoundShape->removeChildShapeByIndex(i);
			}
		}

		for (auto& shape : list_vlp_collisionShapes)
		{
			btTransform t;
			t.setBasis(btMatrix3x3::getIdentity());
			t.setRotation(BulletUtil::QuaternionToBtQuaternion(shape->GetRotation()));
			t.setOrigin(BulletUtil::Vector3ToBtVector3(shape->GetPosition()));
			p_btCompoundShape->addChildShape(t, shape->GetBtCollisionShape());
		}

		p_btCompoundShape->recalculateLocalAabb();

		shape = p_btCompoundShape;
	}

	p_activeShape = shape;
}

ButiEngine::Value_ptr<ButiBullet::PlaneCollisionShape> ButiBullet::PlaneCollisionShape::Create(const ButiEngine::Vector3& arg_direction)
{
    auto output = ButiEngine::make_value<PlaneCollisionShape>();
    output->Initialize(arg_direction);
    return output;
}

ButiBullet::PlaneCollisionShape::PlaneCollisionShape()
{
}

ButiBullet::PlaneCollisionShape::~PlaneCollisionShape()
{
}

bool ButiBullet::PlaneCollisionShape::Initialize()
{
    return Initialize(ButiEngine::Vector3Const::YAxis);
}

bool ButiBullet::PlaneCollisionShape::Initialize(const ButiEngine::Vector3& arg_direction)
{
    return CollisionShape::Initialize(new btStaticPlaneShape(PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_direction.GetNormalize()), 0.0f));
}

ButiEngine::Value_ptr<ButiBullet::CylinderShape> ButiBullet::CylinderShape::Create(const ButiEngine::Vector3& arg_scale)
{
	auto output = ButiEngine::make_value<CylinderShape>();
	output->Initialize(arg_scale);
	return output;
}

ButiBullet::CylinderShape::CylinderShape()
{
}

ButiBullet::CylinderShape::~CylinderShape()
{
}

bool ButiBullet::CylinderShape::Initialize()
{
	return CollisionShape::Initialize();
}

bool ButiBullet::CylinderShape::Initialize(const ButiEngine::Vector3& arg_scale)
{
	return CollisionShape::Initialize(new btCylinderShape(PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_scale)));
}

ButiEngine::Value_ptr<ButiBullet::ConeShape> ButiBullet::ConeShape::Create(const float arg_radius, const float arg_height)
{
	auto output = ButiEngine::make_value<ConeShape>();
	output->Initialize(arg_radius, arg_height);
	return output;
}

ButiBullet::ConeShape::ConeShape()
{
}

ButiBullet::ConeShape::~ConeShape()
{
}

bool ButiBullet::ConeShape::Initialize()
{
	return CollisionShape::Initialize();
}

bool ButiBullet::ConeShape::Initialize(const float arg_radius, const float arg_height)
{
	return CollisionShape::Initialize(new btConeShape(arg_radius, arg_height));
}
