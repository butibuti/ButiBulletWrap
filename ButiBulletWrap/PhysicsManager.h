#ifndef BUTIBULLET_PHYSICSMANAGER_H
#define BUTIBULLET_PHYSICSMANAGER_H


#ifdef BUTIBULLETWRAP_EXPORTS
#define BUTIBULLET_API __declspec(dllexport)
#else
#define BUTIBULLET_API __declspec(dllimport)
#endif
#include"ButiMemorySystem/ButiMemorySystem/ButiList.h"
namespace ButiBullet {

class PhysicsWorld;

extern class PhysicsManager
{
public:
	struct Settings
	{
	};

	BUTIBULLET_API PhysicsManager();
	BUTIBULLET_API virtual ~PhysicsManager();
	BUTIBULLET_API void Initialize(const Settings& arg_settings);
	BUTIBULLET_API void Dispose();
	BUTIBULLET_API void Update();
	BUTIBULLET_API void SetActivePhysicsWorld(ButiEngine::Value_ptr<PhysicsWorld> arg_vlp_world);
	BUTIBULLET_API ButiEngine::Value_ptr<PhysicsWorld> GetActivePhysicsWorld() const;


private:
	ButiEngine::Value_ptr<PhysicsWorld> vlp_activePhysicsWorld;
};

}
#endif // !BUTIBULLET_PHYSICSMANAGER_H