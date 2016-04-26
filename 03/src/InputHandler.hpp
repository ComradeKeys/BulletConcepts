#pragma once

#include <irrlicht.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include "Globals.hpp"

class InputHandler : public irr::IEventReceiver  {

public:
    virtual bool OnEvent(const irr::SEvent &tEvent);
};

