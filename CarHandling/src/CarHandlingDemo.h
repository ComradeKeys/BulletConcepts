/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CARHANDLING_DEMO_H
#define CARHANDLING_DEMO_H

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

class CarHandlingDemo {
public:

	CarHandlingDemo();

	void initPhysics();

	void exitPhysics();

	virtual ~CarHandlingDemo();

	virtual void stepSimulation(float deltaTime);

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}

	virtual void renderScene();

	virtual void physicsDebugDraw(int debugFlags);

	virtual void resetCamera()
	{
		float dist = 5 * 8;
		float pitch = -45;
		float yaw = 32;
		float targetPos[3] = { -0.33, -0.72, 4.5 };
	}

private:
	btDiscreteDynamicsWorld* dynamicsWorld;

	btRaycastVehicle* vehicle;

	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	btRigidBody* createGroundRigidBodyFromShape(btCollisionShape* groundShape);

	btRigidBody* createChassisRigidBodyFromShape(btCollisionShape* chassisShape);

	void addWheels(
		btVector3* halfExtents,
		btRaycastVehicle* vehicle,
		btRaycastVehicle::btVehicleTuning tuning);
};

#endif // FORKLIFT_DEMO_H
