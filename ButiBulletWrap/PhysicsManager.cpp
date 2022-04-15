#include"stdafx.h"
#include"PhysicsWorld.h"
#include"PhysicsManager.h"

ButiBullet::PhysicsManager::PhysicsManager()
{
}

ButiBullet::PhysicsManager::~PhysicsManager()
{
	vlp_activePhysicsWorld->OnDispose(true);
	vlp_activePhysicsWorld = nullptr;
}

void ButiBullet::PhysicsManager::Initialize(const Settings& arg_settings)
{
	if (!vlp_activePhysicsWorld) {
		vlp_activePhysicsWorld = ButiEngine::make_value< PhysicsWorld>();
		vlp_activePhysicsWorld->Initialize();
	}
}

void ButiBullet::PhysicsManager::Dispose()
{
}
void ButiBullet::PhysicsManager::Update()
{
	vlp_activePhysicsWorld->StepSimulation(StepSeconds60FPS);
}

void ButiBullet::PhysicsManager::SetActivePhysicsWorld(ButiEngine::Value_ptr<PhysicsWorld> arg_vlp_world)
{
	vlp_activePhysicsWorld = arg_vlp_world;
}

ButiEngine::Value_ptr<ButiBullet::PhysicsWorld> ButiBullet::PhysicsManager::GetActivePhysicsWorld() const
{
	return vlp_activePhysicsWorld;
}
