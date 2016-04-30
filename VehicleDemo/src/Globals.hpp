#pragma once

#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>
#include <vector>

// Continue while loop?
extern bool done;

// Dimensions of window
extern unsigned short scrnW;
extern unsigned short scrnH;

// Irrlicht visualisation
extern irr::IrrlichtDevice *device;
extern irr::scene::ISceneManager *smgr;
extern irr::video::IVideoDriver *driver;
extern irr::ITimer *timer;
extern irr::scene::ICameraSceneNode *cam;

//Bullet World
extern btDefaultCollisionConfiguration *collisionConfiguration;
extern btBroadphaseInterface *broadphase;
extern btCollisionDispatcher *dispatcher;
extern btSequentialImpulseConstraintSolver *solver;
extern btDiscreteDynamicsWorld *world;
extern std::vector<btRigidBody *> worldObjs;



