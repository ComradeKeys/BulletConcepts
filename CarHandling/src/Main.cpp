#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "CarHandlingDemo.h"

#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"


int main(int argc, char* argv[]) {

    CarHandlingDemo example;

    example.initPhysics();

    while(true) {
	example.stepSimulation(1.f/60.f);
	example.renderScene();
    }


    example.exitPhysics();

    return 0;
}
