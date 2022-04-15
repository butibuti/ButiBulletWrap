#ifndef BUTIBULLETUTIL_H
#define BUTIBULLETUTIL_H
#include"ButiMath/ButiMath.h"
#include"LinearMath/btVector3.h"
#include"LinearMath/btQuaternion.h"
#include"LinearMath/btTransform.h"
namespace ButiBullet {
namespace PhysicsDetail {
class BulletUtil
{
public:
	static const btVector3 Zero;

	static ButiEngine::Vector3 btVector3ToVector3(const btVector3& v)
	{
		return ButiEngine::Vector3(v.getX(), v.getY(), v.getZ());
	}

	static btVector3 Vector3ToBtVector3(const ButiEngine::Vector3& v)
	{
		return btVector3(v.x, v.y, v.z);
	}

	static ButiEngine::Quat btQuaternionToQuaternion(const btQuaternion& q)
	{
		return ButiEngine::Quat(q.getX(), q.getY(), q.getZ(), q.getW());
	}

	static btQuaternion QuaternionToBtQuaternion(const ButiEngine::Quat& q)
	{
		return btQuaternion(q.x, q.y, q.z, q.w);
	}

	static ButiEngine::Matrix4x4 BtTransformToMatrix4x4(const btTransform& t)
	{
		ButiEngine::Matrix4x4 out;
		t.getOpenGLMatrix(reinterpret_cast<btScalar*>(&out));
		return out;
	}

	static btTransform Matrix4x4ToBtTransform(const ButiEngine::Matrix4x4& t)
	{
		btTransform out;
		out.setFromOpenGLMatrix(reinterpret_cast<const btScalar*>(&t));
		return out;
	}


	static void dumpBtVector3(const btVector3& v)
	{
		printf("Vector3\n%f, %f, %f\n", v.x(), v.y(), v.z());
	}

	static void dumpBtTransform(const btTransform& t)
	{
		printf("btTransform\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n",
			t.getBasis().getRow(0).x(), t.getBasis().getRow(0).y(), t.getBasis().getRow(0).z(),
			t.getBasis().getRow(1).x(), t.getBasis().getRow(1).y(), t.getBasis().getRow(1).z(),
			t.getBasis().getRow(2).x(), t.getBasis().getRow(2).y(), t.getBasis().getRow(2).z(),
			t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
	}
};

}
}

#endif // !BUTIBULLETUTIL_H
