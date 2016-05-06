#include "Globals.hpp"
#include "InputHandler.hpp"
bool done = false;
unsigned short scrnW = 800;
unsigned short scrnH = 600;

// Spawning an Irrlicht scene
InputHandler input;
irr::IrrlichtDevice *device = irr::createDevice(irr::video::EDT_OPENGL, irr::core::dimension2d<irr::u32>(scrnW, scrnH), 32, false, false, false, &input);
irr::scene::ISceneManager *smgr = device->getSceneManager();
irr::video::IVideoDriver *driver = device->getVideoDriver();
irr::ITimer *timer = device->getTimer();
irr::scene::ICameraSceneNode *cam = smgr->addCameraSceneNodeFPS(0, 100, 0.01f);

// Spawning a Bullet world
btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
btBroadphaseInterface *broadphase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);
btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver();
btDiscreteDynamicsWorld *world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
std::vector<btRigidBody *> worldObjs;
