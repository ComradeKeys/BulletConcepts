#include "Globals.hpp"
#include "Func.hpp"

/*!
 * Lesson 01 - Hello Bullet
 * Create an irrlicht scene and a bullet world
 * Implement the integration, and build a foundation
 * for the other tutorials
 */
int main(int argc, char *argv[]) {

    // Irrlicht grabs the cursor so it is not in the way of our visualisation
    // Window caption gets set.
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

    // Game loop
    while(!done) {

	// Code to record when the frame began
	deltaTime = timer->getTime() - timeStamp;
	timeStamp = timer->getTime();

	// Step the simulation
	updatePhysics(deltaTime);

	// Irrlicht draws the scene
	driver->beginScene(true, true, irr::video::SColor(0, 15, 192, 252));
	smgr->drawAll();
	driver->endScene();
	device->run();
    }

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
