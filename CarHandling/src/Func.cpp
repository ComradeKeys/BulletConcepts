#include "Func.hpp"
#include "Globals.hpp"

/*!
 * Runs the physics simulation.
 * TDeltaTime tells the simulation how much time has passed since the last frame so the simulation can run independently of the frame rate
 */
void updatePhysics(const irr::u32 &deltaTime) {

    world->stepSimulation(deltaTime * 0.001f, 60);

    // Relay the object's orientation to irrlicht
    for(std::vector<btRigidBody *>::iterator iter = worldObjs.begin(); iter != worldObjs.end(); ++iter) {
	updateRender(*iter);
    }
}

/*!
 * Creates the beginning of the scene, which includes a platform placed at the origin, and a blue cube to fall
 * on top of it
 */
void createScene() {
    clearObjects();
    createBox(btVector3(0.0f, 0.0f, 0.0f), irr::core::vector3df(100.0f, 0.5f, 100.0f), 0.0f, "assets/grass.png");
    //    createBox(btVector3(0.0f, 5.0f, 0.0f), irr::core::vector3df(1.0f, 1.0f, 1.0f), 10.0f, "assets/cube.png");
    createSphere(btVector3(-2.0f, 5.0f, 0.0f), 1.0f, 10.0f, "assets/earth.jpg");
    createCylinder(btVector3(2.0f, 5.0f, 0.0f), btVector3(1.0f, 1.0f, 10.0f), 10.0f, "assets/stone.png");
}

/*!
 * Creates a cube shaped rigid body, and adds a cube onto our scene to match it.
 */
void createBox(const btVector3 &position, const irr::core::vector3df &scale, const btScalar &mass, const char *path) {

    //Visualisation for the rigid body
    irr::scene::ISceneNode *node = smgr->addCubeSceneNode(1.0f);
    node->setScale(scale);
    node->setMaterialFlag(irr::video::EMF_LIGHTING, 1);
    node->setMaterialTexture(0, driver->getTexture(path));

    // Set initial position of the cube
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);

    btDefaultMotionState *MotionState = new btDefaultMotionState(transform);

    // Shape of the rigid body
    btVector3 HalfExtents(scale.X * 0.5f, scale.Y * 0.5f, scale.Z * 0.5f);
    btCollisionShape *Shape = new btBoxShape(HalfExtents);

    // Specify the mass, use it to calculate the local inertia
    btVector3 LocalInertia;
    Shape->calculateLocalInertia(mass, LocalInertia);

    btRigidBody::btRigidBodyConstructionInfo info(mass, MotionState, Shape, LocalInertia);

    // Create the rigid body object
    btRigidBody *rigidBody = new btRigidBody(info);

    // Store a pointer to the irrlicht node so we can update it later
    rigidBody->setUserPointer((void *)(node));

    // Add it to the world
    world->addRigidBody(rigidBody);
    worldObjs.push_back(rigidBody);
}

/*!
 * Creates a sphere shaped rigid body, and adds a sphere onto our scene to match it.
 */
void createSphere(const btVector3 &position, const btScalar &radius, const btScalar &mass, const char *path) {

    //Visualisation for the rigid body
    irr::scene::ISceneNode *node = smgr->addSphereSceneNode(radius, 32);
    node->setMaterialFlag(irr::video::EMF_LIGHTING, 1);
    node->setMaterialTexture(0, driver->getTexture(path));

    // Set the initial position of the object
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);

    btDefaultMotionState *MotionState = new btDefaultMotionState(transform);

    // Create the shape of the rigidbody
    btCollisionShape *Shape = new btSphereShape(radius);

    // Add mass and calculate the local inertia
    btVector3 LocalInertia;
    Shape->calculateLocalInertia(mass, LocalInertia);

    // Create the rigid body object
    btRigidBody *rigidBody = new btRigidBody(mass, MotionState, Shape, LocalInertia);

    // Store a pointer to the irrlicht node so we can update it later
    rigidBody->setUserPointer(static_cast<void *>(node));

    // Add it to the world
    world->addRigidBody(rigidBody);
    worldObjs.push_back(rigidBody);
}

/*!
 * Creates a cylinder shaped rigid body, and adds a cylinder onto our scene to match it.
 */
void createCylinder(const btVector3 &position, const btVector3 &scale, const btScalar &mass, const char *path) {

    //Creating the visualisation for the cylinder
    irr::scene::IAnimatedMesh* cylinder = smgr->getMesh("assets/cylinder.obj");
    irr::scene::ISceneNode *node = smgr->addMeshSceneNode(cylinder);
    node->setScale(irr::core::vector3df(scale.getX(), scale.getY(), scale.getX()));
    node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
    node->setMaterialTexture(0, driver->getTexture(path));
    // Set the initial position of the object
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);

    btDefaultMotionState *MotionState = new btDefaultMotionState(transform);

    // Create the shape for the rigid body
    btCollisionShape *Shape = new btCylinderShape(scale);

    // Add mass and calculate local inertia
    btVector3 LocalInertia;
    Shape->calculateLocalInertia(mass, LocalInertia);

    // Create the rigid body object
    btRigidBody *rigidBody = new btRigidBody(mass, MotionState, Shape, LocalInertia);

    // Store a pointer to the irrlicht node so we can update it later
    rigidBody->setUserPointer(static_cast<void *>(node));
    // Add it to the world
    world->addRigidBody(rigidBody);
    worldObjs.push_back(rigidBody);
}

/*!
 * Passes in bullet rigid body, we extract its transformations and apply it to the visualisation
 */
void updateRender(const btRigidBody *object) {
    irr::scene::ISceneNode *node = static_cast<irr::scene::ISceneNode *>(object->getUserPointer());

    // Set position
    btVector3 Point = object->getCenterOfMassPosition();
    node->setPosition(irr::core::vector3df(static_cast<irr::f32>(Point[0]),
					   static_cast<irr::f32>(Point[1]),
					   static_cast<irr::f32>(Point[2])));

    // Set rotation
    irr::core::vector3df Euler;
    const btQuaternion& tQuat = object->getOrientation();
    irr::core::quaternion q(tQuat.getX(), tQuat.getY(), tQuat.getZ(), tQuat.getW());
    q.toEuler(Euler);
    Euler *= irr::core::RADTODEG;
    node->setRotation(Euler);
}

/*!
 * Removes all objects from the bullet world that we stored in our STL container
 */
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
