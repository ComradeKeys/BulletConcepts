#pragma once

#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>

/*!
 * Storing all our functions here
 */
void createScene();

void createBox(const btVector3 &tPosition, const irr::core::vector3df &tScale, const btScalar &tMass, const char *path = "assets/lava.png");
void updatePhysics(const irr::u32 &tDeltaTime);
void updateRender(const btRigidBody *tObject);
void clearObjects();
