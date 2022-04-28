
#include"stdafx.h"

#include "ButiRendering_Dx12/Header/MeshPrimitive.h""
#include "SoftBody.h"
#include"PhysicsWorld.h"
#include "BulletUtil.h"

#pragma warning(disable: 5033)	// disable warning in bullet headers
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

#pragma warning(default: 5033)
constexpr float DEFAULT_CONFIG_VALUE = 0.1f;
constexpr float DEFAULT_CONFIG_PR = 1.0f;
constexpr float DEFAULT_COLLISION_MARGIN = 0.01f;

void ButiBullet::SoftBody::SetTransform(const ButiEngine::Matrix4x4& arg_transform)
{
    body->transform(PhysicsDetail::BulletUtil::Matrix4x4ToBtTransform(arg_transform));
}

void ButiBullet::SoftBody::SetMass(const float arg_mass)
{
    mass =max(0.0f,arg_mass);
}

std::int32_t ButiBullet::SoftBody::NodeCount() const
{
    return body->m_nodes.size();
}

ButiEngine::Vector3 ButiBullet::SoftBody::NodePosition(const std::int32_t arg_nodeIndex) const
{
    return PhysicsDetail::BulletUtil::btVector3ToVector3(body->m_nodes[arg_nodeIndex].m_x);
}

ButiEngine::Vector3 ButiBullet::SoftBody::NodeVelocity(const std::int32_t arg_nodeIndex) const
{
    return PhysicsDetail::BulletUtil::btVector3ToVector3(body->m_nodes[arg_nodeIndex].m_v);
}

void ButiBullet::SoftBody::SetNodeMass(const std::int32_t arg_nodeIndex, const float arg_mass)
{
    body->setMass(arg_nodeIndex, arg_mass);
}

void ButiBullet::SoftBody::SetLinearStiffness(const float arg_value)
{
    LST = ButiEngine::MathHelper::Clamp<0,1>(arg_value);
}

void ButiBullet::SoftBody::SetAngularStiffness(const float arg_value)
{
    LST = ButiEngine::MathHelper::Clamp<0, 1>(arg_value);
}

void ButiBullet::SoftBody::SetVolumeStiffness(const float arg_value)
{
    LST = ButiEngine::MathHelper::Clamp<0, 1>(arg_value);
}

void ButiBullet::SoftBody::SetPoseMatching(const float arg_value)
{
    LST = ButiEngine::MathHelper::Clamp<0, 1>(arg_value);
}

void ButiBullet::SoftBody::SetCollisionMargin(const float arg_value)
{
    LST = ButiEngine::MathHelper::Clamp<0, 1>(arg_value);
}

void ButiBullet::SoftBody::CreateFromMesh(ButiEngine::ButiRendering::MeshPrimitiveBase* arg_p_mesh, ButiEngine::Value_ptr<PhysicsWorld> arg_vlp_world)
{


    std::vector<btScalar> btVertices;
    std::vector<std::int32_t> btIndices;



    auto numVertices = arg_p_mesh->GetVertexCount();
    btVertices.resize(numVertices * 3);
    for (std::int32_t i = 0; i < numVertices; i++)
    {
        const auto& v =* reinterpret_cast<const ButiEngine::Vertex::VertexInformation::Vertex*>(reinterpret_cast<const std::int8_t*> (arg_p_mesh->GetVertexData())+arg_p_mesh->GetVertexSize()*i);
        btVertices[i * 3 + 0] = v.GetPosition().x;
        btVertices[i * 3 + 1] = v.GetPosition().y;
        btVertices[i * 3 + 2] = v.GetPosition().z;
    }

    auto numIndices = arg_p_mesh->GetIndexCount();
    btIndices.resize(numIndices);
    for (std::int32_t i = 0; i < numIndices; i++)
    {
        btIndices[i] = arg_p_mesh->GetIndexData()[i];
    }
    for (std::int32_t i = 0; i < numIndices / 3; i++)
    {
        std::swap(btIndices[i * 3 + 1], btIndices[i * 3 + 2]);
    }
    body.reset(btSoftBodyHelpers::CreateFromTriMesh(*arg_vlp_world ->GetSoftBodyWorldInfo(), btVertices.data(), btIndices.data(), (std::int32_t)numIndices / 3));

    SetDefaultConfiguration();

    body->generateBendingConstraints(2);

    body->generateClusters(8);
    btSoftRigidDynamicsWorld* worldw = arg_vlp_world->GetBtWorld();
    worldw->addSoftBody(body.get());
}

bool ButiBullet::SoftBody::Raycast(const ButiEngine::Vector3& arg_from, const ButiEngine::Vector3& arg_to, ButiEngine::Vector3* arg_output_p_hitPosition, ButiEngine::Vector3* arg_output_hitNormal) const
{
    if (arg_from==arg_to) return false;

    btSoftBody::sRayCast result;
    bool r = body->rayTest(
        PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_from),
        PhysicsDetail::BulletUtil::Vector3ToBtVector3(arg_to),
        result);
    if (arg_output_p_hitPosition) {
        *arg_output_p_hitPosition = ButiEngine::MathHelper::LerpPosition(arg_from, arg_to, result.fraction);
    }

    if (arg_output_hitNormal) {
        if (result.feature == btSoftBody::eFeature::Face) {
            *arg_output_hitNormal = PhysicsDetail::BulletUtil::btVector3ToVector3(body->m_faces[result.index].m_normal);
        }
    }
    return r;
}

ButiBullet::SoftBody::SoftBody()
    : PhysicsObject(PhysicsObjectType::SoftBody)
    , mass(1.0f)
    , group(1)
    , groupMask(0xffffffff)
    , LST(1.0f)
    , AST(1.0f)
    , VST(1.0f)
    , collisionMargin(0.25f)
    , configLST(DEFAULT_CONFIG_VALUE)
    , MT(0.0f)
    , configVC(DEFAULT_CONFIG_VALUE)
    , configPR(DEFAULT_CONFIG_PR)
{
}

ButiBullet::SoftBody::~SoftBody()
{
}

void ButiBullet::SoftBody::Initialize()
{
    PhysicsObject::Initialize();
}

void ButiBullet::SoftBody::SetDefaultConfiguration()
{
    body->m_cfg.kMT = MT;

    
    body->m_materials[0]->m_kLST = LST;//0.5; // „«(Linear Stiffness Coefficient) (•ÏŒ`‚Ì‚µ‚â‚·‚³)
    body->m_materials[0]->m_kAST = AST;
    body->m_materials[0]->m_kVST = VST;
    //btSoftBody::fMaterial::DebugDraw;

    //m_body->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;//SB“¯Žm‚ÌƒRƒŠƒWƒ‡ƒ“

    if (mass > 0.0f)
    {
        body->setTotalMass(mass, true);
    }

    body->setPose(true, true);
    body->getCollisionShape()->setMargin(collisionMargin);
}
