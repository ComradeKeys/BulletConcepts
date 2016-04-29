#include "Vehicle.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
    Vehicle v;

    v.initPhysics();

    while(true) {
	v.clientMoveAndDisplay();
	v.clientResetScene();
	
    }
    return 0;
}
