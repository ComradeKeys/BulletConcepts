#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include <irrlicht.h>
#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"

#include "CarHandlingDemo.h"
#include "Globals.hpp"
#include "Func.hpp"

int main(int argc, char* argv[]) {

    CarHandlingDemo example;
    device->getCursorControl()->setVisible(0);
    device->setWindowCaption(L"Lesson 01 - Hello Bullet");

    // Set the camera backwards and viewing the origin; setting up the visualisation
    cam->setPosition(irr::core::vector3df(0, 2, -5));
    cam->setTarget(irr::core::vector3df(0, 2, 0));
    smgr->addLightSceneNode(0, irr::core::vector3df(2, 10, -2), irr::video::SColorf(4, 4, 4, 1));

    // Creating our scene
    createScene();

    // Use a timestamp to regulate the frames, and stepping the simulation
    irr::u32 timeStamp = timer->getTime(), deltaTime = 0;

    example.initPhysics();

    // Game loop
    while(!done) {

	// Code to record when the frame began
	deltaTime = timer->getTime() - timeStamp;
	timeStamp = timer->getTime();

	// Step the simulation
	updatePhysics(deltaTime);

	example.stepSimulation(1.f/60.f);
	example.renderScene();

	// Irrlicht draws the scene
	driver->beginScene(true, true, irr::video::SColor(0, 15, 192, 252));
	smgr->drawAll();
	driver->endScene();
	device->run();
    }
    example.exitPhysics();
    // clean up
    clearObjects();
    delete world;
    delete solver;
    delete dispatcher;
    delete broadphase;
    delete collisionConfiguration;
    device->drop();

    return 0;
}
