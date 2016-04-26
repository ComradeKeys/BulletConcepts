#include "InputHandler.hpp"
#include "Func.hpp"

bool InputHandler::OnEvent(const irr::SEvent &tEvent) {

    // Checking the state of the mouse
    if(tEvent.EventType == irr::EET_MOUSE_INPUT_EVENT) {
	switch(tEvent.MouseInput.Event) {
	case irr::EMIE_LMOUSE_PRESSED_DOWN:
	    btVector3 btFrom(cam->getPosition().X, cam->getPosition().Y, cam->getPosition().Z);
	    btVector3 btTo(sin(cam->getRotation().Y * M_PI / 180.0f) * cos(cam->getRotation().X * M_PI / 180.0f) * 50.0f,
			   -1 * sin(cam->getRotation().X * M_PI / 180.0f) * 50.0f,
			   cos(cam->getRotation().Y * M_PI / 180.0f) * cos(cam->getRotation().X * M_PI / 180.0f) * 50.0f);

	    btCollisionWorld::ClosestRayResultCallback res(btFrom, btTo);

	    world->rayTest(btFrom, btTo, res); // m_btWorld is btDiscreteDynamicsWorld

	    if(res.hasHit()){
		printf("Collision at: <%.2f, %.2f, %.2f>\n", res.m_hitPointWorld.getX(), res.m_hitPointWorld.getY(), res.m_hitPointWorld.getZ());
	    }


	    break;
	}
    }
    return false;
}
