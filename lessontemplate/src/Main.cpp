#include <irrlicht.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <cmath>
#include <list>
#include <vector>

// Functions
static void createStartScene();
static void createBox(const btVector3 &tPosition, const irr::core::vector3df &tScale, const btScalar &tMass, const char *path = "assets/lava.png");
static void createSphere(const btVector3 &tPosition, const btScalar &tRadius, const btScalar &tMass, btVector3 vel = btVector3(0, 0, 0), const char *path = "assets/clouds.png");
static void createCylinder(const btVector3 &tPosition, const btVector3 &scale, const btScalar &tMass, btVector3 vel = btVector3(0, 0, 0), const char *path = "assets/lava.png");
static void updatePhysics(const irr::u32 &tDeltaTime);
static void updateRender(const btRigidBody *tObject);
static void clearObjects();
static int  getRandInt(int tMax) { return rand() % tMax; }

// Globals
static bool done = false;
static btSoftRigidDynamicsWorld *world;
static irr::IrrlichtDevice *device;
static irr::video::IVideoDriver *driver;
static irr::scene::ISceneManager *smgr;
static irr::gui::IGUIEnvironment *guienv;
static irr::ITimer *timer;
static std::vector<btRigidBody *> worldObjs;
static irr::scene::ICameraSceneNode *cam;

// Event receiver
class InputHandler : public irr::IEventReceiver  {

public:

    virtual bool OnEvent(const irr::SEvent &tEvent) {

	if(tEvent.EventType == irr::EET_KEY_INPUT_EVENT && !tEvent.KeyInput.PressedDown) {
	    switch(tEvent.KeyInput.Key) {
	    case irr::KEY_ESCAPE:
		done = true;
		break;
	    case irr::KEY_KEY_1:
		
		//btRigidBodyConstructionInfo (btScalar mass, btMotionState *motionState, btCollisionShape *collisionShape, const btVector3 &localInertia=btVector3(0, 0, 0))
		createBox(btVector3(getRandInt(10) - 5.0f, 7.0f, getRandInt(10) - 5.0f), irr::core::vector3df(getRandInt(3) + 0.5f, getRandInt(3) + 0.5f, getRandInt(3) + 0.5f), 1.0f, "assets/gold.png");
		break;

	    case irr::KEY_KEY_2:
		createSphere(btVector3(getRandInt(10) - 5.0f, 7.0f, getRandInt(10) - 5.0f), 0.5f, 1.0f);
		break;

	    case irr::KEY_KEY_3:
		createCylinder(btVector3(getRandInt(10) - 5.0f, 7.0f, getRandInt(10) - 5.0f), btVector3(0.5f, 0.75f, 0.5f), 1.0f);
		break;

	    case irr::KEY_KEY_X:
		createStartScene();
		break;
	    default:
		return false;
		break;
	    }

	    return true;
	}

        // Remember the mouse state
	if(tEvent.EventType == irr::EET_MOUSE_INPUT_EVENT) {
	    switch(tEvent.MouseInput.Event) {
	    case irr::EMIE_LMOUSE_PRESSED_DOWN:

	btVector3 btFrom(cam->getPosition().X, cam->getPosition().Y, cam->getPosition().Z);
	btVector3 btTo(sin(cam->getRotation().Y * M_PI / 180.0f) * cos(cam->getRotation().X * M_PI / 180.0f) * 50.0f,
		       -1 * sin(cam->getRotation().X * M_PI / 180.0f) * 50.0f,
		       cos(cam->getRotation().Y * M_PI / 180.0f) * cos(cam->getRotation().X * M_PI / 180.0f) * 50.0f);

		createSphere(btFrom,
			     0.25f, 1.0f,
			     btTo,
			     "assets/lava.png");

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
};

int main() {

    float scrnW = 800;
    float scrnH = 600;

    // Initialize irrlicht
    InputHandler rec;
    device = createDevice(irr::video::EDT_OPENGL, irr::core::dimension2d<irr::u32>(scrnW, scrnH), 32, false, false, false, &rec);
    guienv = device->getGUIEnvironment();
    timer = device->getTimer();
    smgr = device->getSceneManager();
    driver = device->getVideoDriver();

    device->getCursorControl()->setVisible(0);

    // Initialize bullet
    btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
    btBroadphaseInterface *broadphase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
    btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver();
    world = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

    // Add camera
    cam = smgr->addCameraSceneNodeFPS(0, 100, 0.01f);
    cam->setPosition(irr::core::vector3df(0, 2, -10));
    cam->setTarget(irr::core::vector3df(0, 0, 0));

    // Create text
    irr::gui::IGUISkin *skin = guienv->getSkin();
    skin->setColor(irr::gui::EGDC_BUTTON_TEXT, irr::video::SColor(255, 255, 255, 255));
    guienv->addStaticText(L"1 to spawn a cube\n2 to spawn a sphere\n3 to spawn a cylinder\nHit x to reset", irr::core::rect<irr::s32>(0, 0, 200, 100), false);

    //adding a croshair
    //    guienv->addStaticText(L"+", irr::core::rect<irr::s32>(scrnW / 2, scrnH / 2, (scrnW / 2) + 5, (scrnH / 2) + 10), false);

    // Create the initial scene
    smgr->addLightSceneNode(0, irr::core::vector3df(2, 10, -2), irr::video::SColorf(4, 4, 4, 1));
    createStartScene();

    // Main loop
    irr::u32 timeStamp = timer->getTime(), deltaTime = 0;
    
    while(!done) {

	deltaTime = timer->getTime() - timeStamp;
	timeStamp = timer->getTime();

	updatePhysics(deltaTime);

	driver->beginScene(true, true, irr::video::SColor(255, 20, 0, 0));
	smgr->drawAll();

	irr::core::vector3df btTo(sin(cam->getRotation().Y * M_PI / 180.0f) * cos(cam->getRotation().X * M_PI / 180.0f) * 50.0f,
		       -1 * sin(cam->getRotation().X * M_PI / 180.0f) * 50.0f,
		       cos(cam->getRotation().Y * M_PI / 180.0f) * cos(cam->getRotation().X * M_PI / 180.0f) * 50.0f);

	irr::video::SMaterial m;
	m.Lighting=false;
	driver->setMaterial(m);
	driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());

	driver->draw3DTriangle(irr::core::triangle3df(irr::core::vector3df(1, 1, 1),
						      irr::core::vector3df(1, 0, 1),
						      irr::core::vector3df(0, 0, 0)),
			       irr::video::SColor(0, 0, 255, 0));
	/*
	driver->draw3DLine(cam->getPosition() + irr::core::vector3df(1, .1, 0),
			   btTo,
			   irr::video::SColor(255, 0, 0, 255));
	*/
	guienv->drawAll();
	driver->endScene();
	device->run();

    }

    clearObjects();
    delete world;
    delete solver;
    delete dispatcher;
    delete broadphase;
    delete collisionConfiguration;

    device->drop();

    return 0;
}

// Runs the physics simulation.
// - TDeltaTime tells the simulation how much time has passed since the last frame so the simulation can run independently of the frame rate.
void updatePhysics(const irr::u32 &tDeltaTime) {

    world->stepSimulation(tDeltaTime * 0.001f, 60);

    // Relay the object's orientation to irrlicht
    for(std::vector<btRigidBody *>::iterator iter = worldObjs.begin(); iter != worldObjs.end(); ++iter) {

	updateRender(*iter);
    }
}

// Creates a base box
void createStartScene() {

    clearObjects();
    createBox(btVector3(0.0f, 0.0f, 0.0f), irr::core::vector3df(10.0f, 0.5f, 10.0f), 0.0f, "assets/grass.png");
    createBox(btVector3(-10.0f, 0.0f, 0.0f), irr::core::vector3df(10.0f, 0.5f, 10.0f), 0.0f, "assets/water.png");

}

// Create a box rigid body
void createBox(const btVector3 &tPosition, const irr::core::vector3df &tScale, const btScalar &tMass, const char *path) {

    irr::scene::ISceneNode *node = smgr->addCubeSceneNode(1.0f);
    node->setScale(tScale);
    node->setMaterialFlag(irr::video::EMF_LIGHTING, 1);
    node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    node->setMaterialTexture(0, driver->getTexture(path));

    // Set the initial position of the object
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(tPosition);

    btDefaultMotionState *MotionState = new btDefaultMotionState(transform);

    // Create the shape
    btVector3 HalfExtents(tScale.X * 0.5f, tScale.Y * 0.5f, tScale.Z * 0.5f);
    btCollisionShape *Shape = new btBoxShape(HalfExtents);

    // Add mass
    btVector3 LocalInertia;
    Shape->calculateLocalInertia(tMass, LocalInertia);

    btRigidBody::btRigidBodyConstructionInfo info(tMass, MotionState, Shape, LocalInertia);

    info.m_friction = 100000.0f;
    info.m_restitution = 1.0f;

    // Create the rigid body object
    btRigidBody *rigidBody = new btRigidBody(info);

    // Store a pointer to the irrlicht node so we can update it later
    rigidBody->setUserPointer((void *)(node));

    // Add it to the world
    world->addRigidBody(rigidBody);
    worldObjs.push_back(rigidBody);
}

// Create a sphere rigid body
void createSphere(const btVector3 &tPosition, const btScalar &tRadius, const btScalar &tMass, btVector3 vel, const char *path) {

    irr::scene::ISceneNode *node = smgr->addSphereSceneNode(tRadius, 32);
    node->setMaterialFlag(irr::video::EMF_LIGHTING, 1);
    node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    node->setMaterialTexture(0, driver->getTexture(path));

    // Set the initial position of the object
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(tPosition);

    btDefaultMotionState *MotionState = new btDefaultMotionState(transform);

    // Create the shape
    btCollisionShape *Shape = new btSphereShape(tRadius);

    // Add mass
    btVector3 LocalInertia;
    Shape->calculateLocalInertia(tMass, LocalInertia);

    // Create the rigid body object
    btRigidBody *rigidBody = new btRigidBody(tMass, MotionState, Shape, LocalInertia);

    rigidBody->setLinearVelocity(vel);

    // Store a pointer to the irrlicht node so we can update it later
    rigidBody->setUserPointer(static_cast<void *>(node));
    // Add it to the world
    world->addRigidBody(rigidBody);
    worldObjs.push_back(rigidBody);
}

void createCylinder(const btVector3 &tPosition, const btVector3 &scale, const btScalar &tMass, btVector3 vel, const char *path) {

    irr::scene::IMesh* cylinder = smgr->getMesh("assets/cylinder.obj");
    // make sure cylinder got created

    irr::scene::ISceneNode *node = smgr->addMeshSceneNode(cylinder);
    node->setScale(irr::core::vector3df(scale.getX(), scale.getY(), scale.getX()));
    node->setMaterialFlag(irr::video::EMF_LIGHTING, 1);
    node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    node->setMaterialTexture(0, driver->getTexture(path));

    // Set the initial position of the object
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(tPosition);

    btDefaultMotionState *MotionState = new btDefaultMotionState(transform);

    // Create the shape
    btCollisionShape *Shape = new btCylinderShape(scale);

    // Add mass
    btVector3 LocalInertia;
    Shape->calculateLocalInertia(tMass, LocalInertia);

    // Create the rigid body object
    btRigidBody *rigidBody = new btRigidBody(tMass, MotionState, Shape, LocalInertia);

    rigidBody->setLinearVelocity(vel);

    // Store a pointer to the irrlicht node so we can update it later
    rigidBody->setUserPointer(static_cast<void *>(node));
    // Add it to the world
    world->addRigidBody(rigidBody);
    worldObjs.push_back(rigidBody);
}

// Passes bullet's orientation to irrlicht
void updateRender(const btRigidBody *tObject) {
    irr::scene::ISceneNode *node = static_cast<irr::scene::ISceneNode *>(tObject->getUserPointer());

    // Set position
    btVector3 Point = tObject->getCenterOfMassPosition();
    node->setPosition(irr::core::vector3df(static_cast<irr::f32>(Point[0]),
					   static_cast<irr::f32>(Point[1]),
					   static_cast<irr::f32>(Point[2])));

    // Set rotation
    irr::core::vector3df Euler;
    const btQuaternion& tQuat = tObject->getOrientation();
    irr::core::quaternion q(tQuat.getX(), tQuat.getY(), tQuat.getZ(), tQuat.getW());
    q.toEuler(Euler);
    Euler *= irr::core::RADTODEG;
    node->setRotation(Euler);
}

// Removes all objects from the world
void clearObjects() {

    for(std::vector<btRigidBody *>::iterator iter = worldObjs.begin(); iter != worldObjs.end(); ++iter) {
	btRigidBody *object = *iter;

	// Delete irrlicht node
	irr::scene::ISceneNode *node = static_cast<irr::scene::ISceneNode *>(object->getUserPointer());
	node->remove();

	// Remove the object from the world
	world->removeRigidBody(object);

	// Free memory
	delete object->getMotionState();
	delete object->getCollisionShape();
	delete object;
    }

    worldObjs.clear();
}
