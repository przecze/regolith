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

double BOX_R = 0.4;
double BOX_H = 0.2;

#include "Regolith.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"

#include "packgen/gen_pack.h"

#include <iostream>
#include <chrono>
#include <random>

struct ResponseAngle : public CommonRigidBodyBase
{
	ResponseAngle(struct GUIHelperInterface* helper,
			          regolith::RegolithProperties regolith_properties)
		: CommonRigidBodyBase(helper),
			regolith(regolith_properties, 10)
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
	regolith::Regolith regolith;
};

void ResponseAngle::initPhysics()
{
	BOX_H = regolith.properties.maxRadius * 20;
	BOX_R = BOX_H;
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,-1.,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btCylinderShape(btVector3(btScalar(1.*BOX_R), btScalar(BOX_H/2), 1.*BOX_R));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -BOX_H/2, 0));
	auto ground = createRigidBody(0., groundTransform, groundShape, btVector4(0, 0, 1, 1));
	ground->setRestitution(0.);
	ground->setFriction(1);

	auto sizes_count = regolith.grain_radii.size();
	double p[sizes_count];
	std::fill_n(p, sizes_count, 1./sizes_count);
	double* r = &regolith.grain_radii[0];
	PG::NG* ng = new PG::GeneralNG(r,
	                               p,
	                               sizes_count);
	PG::Grid3d dom;
	PG::Container* container = new PG::Cylinder({0.0, 0.0, 0.0},
																							{0.0, BOX_H, 0.0},
																							BOX_R);

	PG::SpherePack* pack = new PG::SpherePack();
	PG::SpherePackStat result = PG::GenerateSpherePack(container, ng, &dom, pack);

	for(auto s: pack->s) {
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(s.x, s.y, s.z));
		regolith.createGrain(this, transform, s.r);
	}

  // TODO : is it required for sth?
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
  //for(auto shape : regolithProperties.collisionShapes) {
  //  m_collisionShapes.push_back(shape);
  //}
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
		for(auto i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i) {
			auto body = m_dynamicsWorld->getCollisionObjectArray()[i];
			if(body->getWorldTransform().getOrigin().getY() < - (BOX_H/2. + 0.01)) {
				m_dynamicsWorld->removeRigidBody(btRigidBody::upcast(body));
				const double transparent[4] = {0.,0.,0.,0.};
				m_guiHelper->changeRGBAColor(btRigidBody::upcast(body)->getUserIndex(), transparent);
			}
		}
		double max_y = 0.;
		for(auto i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i) {
			auto body = m_dynamicsWorld->getCollisionObjectArray()[i];
			max_y = std::max(max_y, double(body->getWorldTransform().getOrigin().getY()));
		}
		std::cout<<"h = "<<max_y<<std::endl;
		std::cout<<"h/r = "<<max_y/2./BOX_R<<std::endl;
	}
  if(m_dynamicsWorld) {
    m_dynamicsWorld->stepSimulation(deltaTime);
  }
}

CommonExampleInterface* CreateFunc(CommonExampleOptions& options)
{
	auto config = YAML::LoadFile("config.yaml");
	auto properties = [&config]{
		try {
			std::cout<<"Regolith configuration found in config.yaml"<<std::endl;
			return regolith::loadPropertiesFromYaml(config["regolith"]);
		}
		catch(YAML::InvalidNode) {
			std::cout<<"No regolith configuration found in config.yaml. "
								 "Loading regolith.yaml"<<std::endl;
			return regolith::loadPropertiesFromFile("regolith.yaml");
		}}();
	return new ResponseAngle(options.m_guiHelper, properties);
}

B3_STANDALONE_EXAMPLE(CreateFunc)
