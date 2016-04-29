#include "Vehicle.hpp"

int rightIndex(0);
int upIndex(1);
int forwardIndex(2);
btVector3 wheelDirectionCS0(0, -1, 0);
btVector3 wheelAxleCS(-1, 0, 0);

const int maxProxies = 32766;
const int maxOverlap = 65535;

float gEngineForce = 0.f;
float gBreakingForce = 0.f;
float maxEngineForce = 1000.f;
float maxBreakingForce = 100.f;
float gVehicleSteering = 0.f;
float steeringIncrement = 0.4f;
float steeringClamp = 0.3f;
float wheelRadius = 0.75f;
float wheelWidth = 0.4f;
float wheelFriction = 1000;
float suspensionStiffness = 20.0f;
float suspensionDamping= 2.3f;
float suspensionCompression = 4.4f;
float rollInfluence = 0.1f;

btScalar suspensionRestLength(2.0);

#define CUBE_HALF_EXTENTS 1

Vehicle::Vehicle() :
    carChassis(0),
    indexVertexArrays(0),
    vertices(0)
{
    vehicle = 0;
    wheelShape = 0;
}

Vehicle::~Vehicle() {
    int i;
    for(i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
	btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[i];
	btRigidBody *body = btRigidBody::upcast(obj);
	if(body and body->getMotionState()) {
	    delete body->getMotionState();
	}
	dynamicsWorld->removeCollisionObject(obj);
	delete obj;
    }

    for (int j = 0; j < collisionShapes.size(); j++) {
	btCollisionShape *shape = collisionShapes[j];
	delete shape;
    }

    delete indexVertexArrays;
    delete vertices;
    delete dynamicsWorld;
    delete vehicleRayCaster;
    delete vehicle;
    delete wheelShape;
    delete constraintSolver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
}

void Vehicle::clientMoveAndDisplay() {
    {
	int wheelIndex = 2;
	vehicle->applyEngineForce(gEngineForce, wheelIndex);
	vehicle->setBrake(gBreakingForce, wheelIndex);
	wheelIndex = 3;
	vehicle->applyEngineForce(gEngineForce, wheelIndex);
	vehicle->setBrake(gBreakingForce, wheelIndex);

	wheelIndex = 0;
	vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
	wheelIndex = 1;
	vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
    }

    //    float dt = getDeltaTimeMicroseconds() * 0.000001f;
    float dt = .000001f;

    if(dynamicsWorld) {
	bool idle = true;
	//during idle mode, just run 1 simulation step maximum
	int maxSimSubSteps = idle ? 1 : 2;
	if(idle) {
	    dt = 1.0f/420.f;

	    int numSimSteps = dynamicsWorld->stepSimulation(dt, maxSimSubSteps);
	}
    }
}	

void Vehicle::clientResetScene() {

    gVehicleSteering = 0.f;
    carChassis->setCenterOfMassTransform(btTransform::getIdentity());
    carChassis->setLinearVelocity(btVector3(0, 0, 0));
    carChassis->setAngularVelocity(btVector3(0, 0, 0));
    dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(carChassis->getBroadphaseHandle(), dynamicsWorld->getDispatcher());

    if(vehicle) {
	vehicle->resetSuspension();
	for(int i = 0; i < vehicle->getNumWheels(); i++) {
	    //sync wheels with interpolated chassis
	    vehicle->updateWheelTransform(i, true);
	}
    }

}

void Vehicle::displayCallback() {
    return;
}	

//we are going to need input handlers later

void Vehicle::renderMe() {
    return;
}	

void Vehicle::initPhysics() {
    btCollisionShape *groundShape = new btBoxShape(btVector3(50, 3, 50));
    collisionShapes.push_back(groundShape);
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btVector3 worldMin(-1000, -1000, -1000);
    btVector3 worldMax(1000, 1000, 1000);
    overlappingPairCache = new btAxisSweep3(worldMin, worldMax);
    constraintSolver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, constraintSolver, collisionConfiguration);

    btTransform tr;
    tr.setIdentity();

    int i;
    const float TRIANGLE_SIZE=20.f;

    //create a triangle-mesh ground
    int vertStride = sizeof(btVector3);
    int indexStride = 3 * sizeof(int);
    const int NUM_VERTS_X = 20;
    const int NUM_VERTS_Y = 20;
    const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;

    const int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);

    vertices = new btVector3[totalVerts];
    int *gIndices = new int[totalTriangles * 3];

    for(i = 0; i < NUM_VERTS_X; i++) {
	for(int j = 0; j < NUM_VERTS_Y; j++) {
	    float wl = .2f;
	    float height = 0.f;

	    vertices[i + j * NUM_VERTS_X].setValue(
						   (i - NUM_VERTS_X * 0.5f) * TRIANGLE_SIZE,
						   height,
						   (j - NUM_VERTS_Y * 0.5f) * TRIANGLE_SIZE);
	}
    }

    int index = 0;
    for (i = 0; i < NUM_VERTS_X - 1; i++) {
	for(int j = 0; j < NUM_VERTS_Y - 1; j++) {
	    gIndices[index++] = j * NUM_VERTS_X + 1;
	    gIndices[index++] = j * NUM_VERTS_X + i + 1;
	    gIndices[index++] = (j + 1) * NUM_VERTS_X + 1;

	    gIndices[index++] = j * NUM_VERTS_X + i;
	    gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
	    gIndices[index++] = (j + 1) * NUM_VERTS_X + i;
	    
	}
    }

    indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
						       gIndices,
						       indexStride,
						       totalVerts,
						       (btScalar*) &vertices[0].x(), vertStride);

    bool useQuantizedAabbCompression = true;
    groundShape = new btBvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression);

    tr.setOrigin(btVector3(0, -4.5f, 0));

    localCreateRigidBody(0, tr, groundShape);

    btCollisionShape *chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
    collisionShapes.push_back(chassisShape);

    btCompoundShape *compound = new btCompoundShape();
    collisionShapes.push_back(compound);
    btTransform localTrans;
    localTrans.setIdentity();

    localTrans.setOrigin(btVector3(0, 1, 0));

    compound->addChildShape(localTrans, chassisShape);

    tr.setOrigin(btVector3(0, 0.f, 0));

    carChassis = localCreateRigidBody(800, tr, compound);

    wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

    clientResetScene();

    //create vehicle
    {
	vehicleRayCaster = new btDefaultVehicleRaycaster(dynamicsWorld);
	vehicle = new btRaycastVehicle(tuning, carChassis, vehicleRayCaster);

	//never deactivate the vehicle
	carChassis->setActivationState(DISABLE_DEACTIVATION);
	dynamicsWorld->addVehicle(vehicle);

	float connectionHeight = 1.2f;

	bool isFrontWheel = true;

	//choose coordinate system
	vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);


		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);

		vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,tuning,isFrontWheel);

		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);

		vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,tuning,isFrontWheel);

		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);

		isFrontWheel = false;
		vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,tuning,isFrontWheel);

		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);

		vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,tuning,isFrontWheel);		

		for(int i = 0; i < vehicle->getNumWheels(); i++) {
		    btWheelInfo &wheel = vehicle->getWheelInfo(i);
		    wheel.m_suspensionStiffness = suspensionStiffness;
		    wheel.m_wheelsDampingRelaxation = suspensionDamping;
		    wheel.m_wheelsDampingCompression = suspensionCompression;

		    wheel.m_frictionSlip = wheelFriction;
		    wheel.m_rollInfluence = rollInfluence;
		}

    }
    
}

