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
#define ARRAY_SIZE_X 6
#define ARRAY_SIZE_Z 6

constexpr double BOX_W = 0.4;
constexpr double BOX_H = 0.2;

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"

#include <iostream>
#include <chrono>
#include <random>
namespace {
struct RegolithProperties {
  double maxRadius;
  double minRadius;
  double resitution;
  double friction;
  double rollingFriction;
  double mass;
  std::vector<btCollisionShape*> collisionShapes;
};

double sizeInverseDistribution(double p, const RegolithProperties & regolithProperties) {
  assert(p <=1.);
  assert(p >=0.);
  return regolithProperties.maxRadius * std::exp( std::log(regolithProperties.minRadius / regolithProperties.maxRadius) * p);
}

const auto regolithProperties = []{
  auto properties = RegolithProperties{};
  properties.maxRadius = 0.04;
  properties.minRadius = 0.005;
  properties.resitution = 0.1;
  properties.friction = 1.;
  properties.rollingFriction = 1.;
  properties.mass = 0.3;
  for(int i = 0; i <= 1; ++i) {
    properties.collisionShapes.push_back(new btSphereShape(sizeInverseDistribution(0., properties)));
  }
  return properties;}();


struct ResponseAngle : public CommonRigidBodyBase
{
	ResponseAngle(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~ResponseAngle() {}
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

void ResponseAngle::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,-1.,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btCylinderShape(btVector3(btScalar(1.*BOX_W), btScalar(BOX_H/2), 1.*BOX_W));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -BOX_H/2, 0));
  auto ground = createRigidBody(0., groundTransform, groundShape, btVector4(0, 0, 1, 1));
  ground->setRestitution(0.);
  ground->setFriction(1);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
  for(auto shape : regolithProperties.collisionShapes) {
    m_collisionShapes.push_back(shape);
  }
}

void ResponseAngle::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

void ResponseAngle::stepSimulation(float deltaTime)
{
  constexpr auto update_steps = 60;
  constexpr auto layers_per_update = 3;
  static int count = 0;
  static int grains_count = 0;
  static bool stop_adding = false;

  ++count;
  if (count % update_steps == 0) {
    count = 0;
    if(not stop_adding) {

      btTransform startTransform;
      startTransform.setIdentity();

      auto random_engine = std::mt19937{std::random_device{}()};
      auto vertical_vel_distribution = std::uniform_real_distribution<btScalar>{-0.1, 0.1};
      auto uniform_distribution = std::uniform_int_distribution<int>{0, 1};
      for (int k = 0; k < layers_per_update; k++)
      {
        for (int i = 0; i < ARRAY_SIZE_X; i++)
        {
          for (int j = 0; j < ARRAY_SIZE_Z; j++)
          {
            startTransform.setOrigin(btVector3(
              btScalar(-BOX_W/2. + (BOX_W)/(ARRAY_SIZE_X-1) * i),
              btScalar(2*BOX_H + (1. + k) * 2. * regolithProperties.maxRadius),
              btScalar(-BOX_W/2. + (BOX_W)/(ARRAY_SIZE_Z-1) * j)));

            auto body = createRigidBody(regolithProperties.mass, startTransform, regolithProperties.collisionShapes[uniform_distribution(random_engine)], btVector4(0,0,1,1), btVector3(0,-1.,0));
            body->setFriction(regolithProperties.friction);
            body->setRestitution(regolithProperties.resitution);
            body->setRollingFriction(regolithProperties.rollingFriction);
            body->setLinearVelocity(btVector3(vertical_vel_distribution(random_engine), 0., vertical_vel_distribution(random_engine)));
          }
        }
      }
      auto currrent_count = m_dynamicsWorld->getNumCollisionObjects();
      if(currrent_count + 10 < grains_count) {
        stop_adding = true;
      } else {
        grains_count = currrent_count;
      }
      std::cout<<"ADDED: "<<m_dynamicsWorld->getNumCollisionObjects()<<std::endl;
      for(auto i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i) {
        auto body = m_dynamicsWorld->getCollisionObjectArray()[i];
        if(body->getWorldTransform().getOrigin().getY() < - (BOX_H/2. + 0.01)) {
          m_dynamicsWorld->removeRigidBody(btRigidBody::upcast(body));
        }
         
      }

      std::cout<<"REMOVED: "<<m_dynamicsWorld->getNumCollisionObjects()<<std::endl;
      m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    } else {
      for(auto i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i) {
        auto body = m_dynamicsWorld->getCollisionObjectArray()[i];
        if(body->getWorldTransform().getOrigin().getY() < - (BOX_H/2. + 0.01)) {
          m_dynamicsWorld->removeRigidBody(btRigidBody::upcast(body));
        }
         
      }
      double max_y = 0.;
      for(auto i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i) {
        auto body = m_dynamicsWorld->getCollisionObjectArray()[i];
        max_y = std::max(max_y, double(body->getWorldTransform().getOrigin().getY()));
      }
      std::cout<<"max_y = "<<max_y<<std::endl;
    }

  }
  if(m_dynamicsWorld) {
    m_dynamicsWorld->stepSimulation(deltaTime);
  }
}
}

CommonExampleInterface* CreateFunc(CommonExampleOptions& options)
{
	return new ResponseAngle(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(CreateFunc)
