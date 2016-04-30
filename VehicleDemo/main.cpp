#include "VehicleDemo.h"
#include <btBulletDynamicsCommon.h>

int main(int argc,char** argv) {

    VehicleDemo* vehicleDemo = new VehicleDemo;

    vehicleDemo->initPhysics(); 

    while(true) {
	vehicleDemo->clientMoveAndDisplay();
    }
}

