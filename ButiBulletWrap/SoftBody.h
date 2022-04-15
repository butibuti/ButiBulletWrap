#ifndef BUTIBULLET_SOFTBODY_H
#define BUTIBULLET_SOFTBODY_H
#include"Common.h"
#include"PhysicsObject.h"
#include<memory>
namespace ButiEngine {
class MeshResource;
}
namespace ButiBullet {

class PhysicsWorld;

class SoftBody: public PhysicsObject
{
public:
    BUTIBULLET_API void SetTransform(const ButiEngine:: Matrix4x4& arg_transform);
    BUTIBULLET_API void SetMass(const float arg_mass);
    BUTIBULLET_API std::int32_t NodeCount() const;

    BUTIBULLET_API ButiEngine::Vector3 NodePosition(const std::int32_t arg_nodeIndex) const;
    BUTIBULLET_API ButiEngine::Vector3 NodeVelocity(const std::int32_t arg_nodeIndex) const;

    BUTIBULLET_API void SetNodeMass(const std::int32_t arg_nodeIndex,const float arg_mass);



    BUTIBULLET_API void SetLinearStiffness(const float arg_value);
    BUTIBULLET_API void SetAngularStiffness(const float arg_value);
    BUTIBULLET_API void SetVolumeStiffness(const float arg_value);
    BUTIBULLET_API void SetPoseMatching(const float arg_value);
    BUTIBULLET_API void SetCollisionMargin(const float arg_value);

    BUTIBULLET_API void CreateFromMesh(ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh,ButiEngine::Value_ptr<PhysicsWorld> arg_vlp_world);
    BUTIBULLET_API bool Raycast(const ButiEngine::Vector3& arg_from, const ButiEngine::Vector3& arg_to, ButiEngine::Vector3* arg_output_p_hitPosition, ButiEngine::Vector3* arg_output_hitNormal) const;

    BUTIBULLET_API SoftBody();
    BUTIBULLET_API virtual ~SoftBody();
    BUTIBULLET_API void Initialize();

private:
    void SetDefaultConfiguration();

    std::unique_ptr<btSoftBody> body;
    float mass;
    std::uint32_t group;
    std::uint32_t groupMask;

    float LST;
    float AST;
    float VST;

    float collisionMargin;


    float configLST;
    /// 
    float MT;
    float configVC;
    float configPR;


    friend class PhysicsWorld;
};
}

#endif // !BUTIBULLET_SOFTBODY_H

