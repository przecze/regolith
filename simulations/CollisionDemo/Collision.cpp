/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"

#include <iostream>
#include <chrono>
#include <thread>

struct CollisionDemo : public CommonRigidBodyBase
{
	CollisionDemo(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~CollisionDemo() {}
	virtual void initPhysics();
	virtual void renderScene();
	virtual void stepSimulation(float deltaTime);
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void CollisionDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,-0.5,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	btCollisionShape* groundShape = new btBoxShape(btVector3(10.,1,10.));

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0,-1.,0));
  auto g = createRigidBody(0., groundTransform, groundShape, btVector4(0, 0, 1, 1));
  g->setRestitution(1.);

  constexpr auto SPHERE_R = 0.3;
  constexpr auto DROP_H = 1.5;
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btCollisionShape* colShape = new btSphereShape(SPHERE_R);

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;

		btScalar mass(1.f);

    startTransform.setOrigin(btVector3(0, DROP_H, 0));
    auto b = createRigidBody(mass, startTransform, colShape, btVector4(0,0,1,1), btVector3(0,-1.,0));
    b->setRestitution(0.5);

    startTransform.setOrigin(btVector3(0, 2*DROP_H, 0));
    b = createRigidBody(mass, startTransform, colShape, btVector4(0,0,1,1), btVector3(0,-1.,0));
    b->setRestitution(0.5);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void CollisionDemo::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

void CollisionDemo::stepSimulation(float deltaTime)
{
  using namespace std::chrono;
  static auto last = steady_clock::now();
  auto current = steady_clock::now();
  auto elapsed = current - last;
  last = current;
  if(m_dynamicsWorld) {
    m_dynamicsWorld->stepSimulation(deltaTime);
  }
  auto probeIndex = m_dynamicsWorld->getNumCollisionObjects() - 1;
  btCollisionObject* obj=m_dynamicsWorld->getCollisionObjectArray()[probeIndex];
  btRigidBody* body=btRigidBody::upcast(obj);
  btTransform trans;
  if(body&&body->getMotionState()) {
    body->getMotionState()->getWorldTransform(trans);
  } else {
    trans=obj->getWorldTransform();
  }
  std::cout<<float(trans.getOrigin().getX())<<" "
      <<float(trans.getOrigin().getY())<<" "
      <<float(trans.getOrigin().getZ())<<" "
      <<std::chrono::duration_cast<milliseconds>(elapsed).count()<<" ms"<<std::endl;
  std::this_thread::sleep_for(milliseconds(1000/60));
}

CommonExampleInterface* CreateFunc(CommonExampleOptions& options)
{
	return new CollisionDemo(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(CreateFunc)
