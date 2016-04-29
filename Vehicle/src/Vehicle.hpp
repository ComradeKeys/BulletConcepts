#pragma once

#include <irrlicht.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <btBulletDynamicsCommon.h>
#include <bullet/LinearMath/btQuickprof.h>

//btRigidBody* localCreateRigidBody(btScalar mass,const btTransform& startTrans,btCollisionShape* colShape);

class Vehicle {

public:
    btDiscreteDynamicsWorld *dynamicsWorld;

    btClock clock;

    btRigidBody *carChassis;

    btAlignedObjectArray<btCollisionShape*> collisionShapes;

    btBroadphaseInterface *overlappingPairCache;

    btCollisionDispatcher *dispatcher;

    btConstraintSolver *constraintSolver;

    btDefaultCollisionConfiguration *collisionConfiguration;

    btTriangleIndexVertexArray *indexVertexArrays;

    btVector3 *vertices;


    btRaycastVehicle::btVehicleTuning tuning;
    btVehicleRaycaster *vehicleRayCaster;
    btRaycastVehicle *vehicle;
    btCollisionShape *wheelShape;

    Vehicle();
    virtual ~Vehicle();

    virtual void clientMoveAndDisplay();

    virtual void clientResetScene();
    virtual void displayCallback();

    //we are going to need input handlers later

    btRigidBody* localCreateRigidBody(btScalar mass,const btTransform& startTrans,btCollisionShape* colShape)
    {
	btVector3 inertia(0,0,0);
	if (mass)
	    colShape->calculateLocalInertia(mass,inertia);
	btRigidBody::btRigidBodyConstructionInfo rbci(mass,0,colShape,inertia);
	rbci.m_startWorldTransform = startTrans;

	btRigidBody* body = new btRigidBody(rbci);
	dynamicsWorld->addRigidBody(body);
	return body;

    }
    void renderMe();

    void initPhysics();

};
