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
    device->setWindowCaption(L"Lesson 03 - Intro to Raycasting");

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

	//Visualize the raycast for the camera. The line needs to only be as long as the ray cast that way we are not blazing through objects and it really helps show the concept of ray casting, and it is a more realistic laser sight

	// Need the name for this mathmatical formula
	irr::core::vector3df rayEnd(sin(cam->getRotation().Y * M_PI / 180.0f) * cos(cam->getRotation().X * M_PI / 180.0f) * 50.0f,
		       -1 * sin(cam->getRotation().X * M_PI / 180.0f) * 50.0f,
		       cos(cam->getRotation().Y * M_PI / 180.0f) * cos(cam->getRotation().X * M_PI / 180.0f) * 50.0f);

	// Reset the transformation matrix for the raycast
	irr::video::SMaterial m;
	m.Lighting=false;
	driver->setMaterial(m);
	driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());

	//Draw our laser sight
	driver->draw3DLine(cam->getPosition() + irr::core::vector3df(.1, .01, 0),
			   rayEnd,
			   irr::video::SColor(255, 255, 0, 0));
	
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
