#pragma once

#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>

/*!
 * Storing all our functions here
 */
void createScene();

/*!
 * Functions used to create bullet meshes
 */
void createBox(const btVector3 &position, const irr::core::vector3df &scale, const btScalar &mass, const char *path = "assets/lava.png");
void createSphere(const btVector3 &position, const btScalar &radius, const btScalar &mass, const char *path = "assets/clouds.png");
void createCylinder(const btVector3 &position, const btVector3 &scale, const btScalar &mass, const char *path = "assets/lava.png");

void updatePhysics(const irr::u32 &deltaTime);
void updateRender(const btRigidBody *object);
void clearObjects();
