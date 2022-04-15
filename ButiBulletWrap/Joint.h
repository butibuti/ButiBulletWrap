#ifndef BUTIBULLET_JOINT_H
#define BUTIBULLET_JOINT_H
#include"Common.h"
namespace ButiBullet {
class Joint :public ButiEngine::enable_value_from_this<Joint>
{
public:

    ButiEngine::Value_ptr< PhysicsWorld> GetPhysicsWorld() const { return vlp_world; }

    BUTIBULLET_API void RemoveFromPhysicsWorld();


    BUTIBULLET_API Joint();

private:
    virtual void RemoveFromBtWorld() = 0;

    ButiEngine::Value_ptr< PhysicsWorld> vlp_world;
    bool removing;

    friend class PhysicsWorld;
};

}

#endif // !BUTIBULLET_JOINT_H
