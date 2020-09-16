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

#include "utils.h"
#include "Regolith.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include <iostream>
#include <chrono>
#include <random>

namespace {
constexpr double BOX_DIAMETER = 1.2; // m
constexpr double BOX_H = 0.7; // m
constexpr double probeVelocity = -.1; // m/s
constexpr btScalar probeRadius = 0.07; // m
constexpr btScalar pressure = 60000; // Pa
constexpr btScalar pressurePlateThickness = 0.05; // m

double correction_factor_old(double Dr, double Rd) {
  // I think I read somewhere that this version can be used for BC3.
  // But the values are really small (E-6)
  // This is CF BC1 formula from Butlanska et. al 2011 (Cone Penetration Tests in Virtual Chamber)
  // But with -b instead of b in the exponent (which was supposed to work in BC3 case, I don't remember where I read it)
  auto a = 9*0.00001*std::pow(Rd, 2.02);
  auto b = -0.565*std::log(Rd)+2.59;
  return a*std::pow(100*Dr, -b);
}
double correction_factor(double Dr, double Rd) {
  // Mayne & Kulhawy 1991
  // As given in Butlanska et al. 2010 (Size effects on a virtual calibration chamber)
  return std::pow((Rd-1)/70, -Dr/2.);
}



auto regolith = []{
  auto properties = load_properties_from_file("regolith.yaml");
  return Regolith{properties, 10};
}();

  
struct ConePenetrationTest : public CommonRigidBodyBase
{
	ConePenetrationTest(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~ConePenetrationTest() {}
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

  btRigidBody* probe;
  btRigidBody* pressurePlate;
  void createProbe();
  void createPressurePlate();

  int removeGrains();
  void addGrains(int layers, bool initial);
  void resetVelocities();

  enum {
    FILLING_PHASE,
    STABILIZATION_PHASE,
    PRESSURE_PHASE,
    PENETRATION_PHASE,
    FINISHED_PHASE
  } phase = FILLING_PHASE;

  double correctionFactor = 0.;

  btScalar dt = 1./60.; // s
  double update_time = 1.; // s
  void rescaleTime(float rescaleFactor) {
    std::cout<<"Rescaling time by factor "<<rescaleFactor<<std::endl;
    dt/=rescaleFactor;
    update_time/=rescaleFactor;
  }

};

void ConePenetrationTest::initPhysics()
{
	m_guiHelper->setUpAxis(1);
	const double transparent[4] = {0.,0.,0.,0.};

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(BOX_DIAMETER), btScalar(BOX_H/2), btScalar(BOX_DIAMETER)));

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -BOX_H/2, 0));
	auto ground = createRigidBody(0., groundTransform, groundShape, btVector4(0, 0, 1, 0));

  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3{0., -BOX_H, 0.});
  auto wall_thickness = 0.01;
  auto towerShape = utils::BuildTowerCompoundShape(
                             btVector3{(BOX_DIAMETER+wall_thickness)/(std::sqrt(5 + 2.*std::sqrt(5))),
                                        BOX_H*2.,
                                        wall_thickness},
                             1);
	m_collisionShapes.push_back(towerShape);
  auto tower = createRigidBody(0., groundTransform, towerShape, btVector4(0, 0, 0, 0));

  btTransform startTransform;
  startTransform.setIdentity();
  auto random_engine = std::mt19937{std::random_device{}()};
  addGrains(BOX_H/(2.*regolith.properties.maxRadius), true);
  std::cout<<"ADDED: "<<m_dynamicsWorld->getNumCollisionObjects()<<std::endl;

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	m_guiHelper->changeRGBAColor(ground->getUserIndex(), transparent);
	m_guiHelper->changeRGBAColor(tower->getUserIndex(), transparent);
}

void ConePenetrationTest::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

void ConePenetrationTest::stepSimulation(float deltaTime)
{
  using namespace std::chrono;
  static auto last = steady_clock::now();
  static int steps_since_last_update = 0;
  static int grains_count = 0;
  {
    auto numSteps = m_dynamicsWorld->stepSimulation(deltaTime, 1, dt); // note: maxSubSteps = 1 is passed implicitly here as default argument. Therefore we will do only one step (1/60 s) not the whole deltaTime ( = 0.1 s in gui demos)
    steps_since_last_update += numSteps;
  }
  if(steps_since_last_update*dt > update_time) {
    auto current = steady_clock::now();
    auto elapsed = current - last;
    last = current;
    std::cout<<"last step took: "<<std::chrono::duration_cast<milliseconds>(elapsed).count()<<" ms"<<std::endl;
    if(phase == FILLING_PHASE) {
      constexpr auto layers_per_update = 2;
      int removed = removeGrains();
      auto removed_limit = BOX_DIAMETER*BOX_DIAMETER
                             /regolith.properties.maxRadius
                             /regolith.properties.maxRadius
                             /8;
      if(removed > removed_limit) {
        std::cout<<"Entering STABILIZATION_PHASE"<<std::endl;
        phase = STABILIZATION_PHASE;
      } else {
        addGrains(layers_per_update, false);
        std::cout<<"Grains count: "<<m_dynamicsWorld->getNumCollisionObjects()<<std::endl;
      }
    } else if (phase == STABILIZATION_PHASE) {
        int removed = removeGrains();
        resetVelocities();
        if(removed == 0) {
          phase=PRESSURE_PHASE;
          rescaleTime(4.);
          std::cout<<"Entering PRESSURE_PHASE"<<std::endl;
          createPressurePlate();
          std::cout<<"Create pressure plate"<<std::endl;
        }
    } else if (phase == PRESSURE_PHASE) {
        std::cout<<"plate y: "<<pressurePlate->getWorldTransform().getOrigin().getY()<<" v: "<<pressurePlate->getLinearVelocity().getY()<<std::endl;
        if(abs(pressurePlate->getLinearVelocity().getY())<0.01) {
          std::cout<<"Entering PENETRATION_PHASE"<<std::endl;
          phase = PENETRATION_PHASE;
          createProbe();
          auto h = pressurePlate->getWorldTransform().getOrigin().getY()-pressurePlateThickness/2;
          auto V = h*BOX_DIAMETER/2.*BOX_DIAMETER/2.*SIMD_PI;
          double grains_mass = 0.;
          double grains_volume = 0.;
          for(auto i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i) {
            auto body = m_dynamicsWorld->getCollisionObjectArray()[i];
            if(body->getCollisionShape()->getShapeType() == SPHERE_SHAPE_PROXYTYPE) {
              grains_mass+=1./btRigidBody::upcast(body)->getInvMass();
              auto r =dynamic_cast<btSphereShape*>(body->getCollisionShape())->getRadius();
              grains_volume+=4./3.*SIMD_PI*r*r*r;
            }
          }
          std::cout<<"void ratio: "<<(V-grains_volume)/grains_volume<<std::endl;
          std::cout<<"\% of volume used: "<<grains_volume/V*100.<<std::endl;
          auto relativeDensity = ((grains_mass/V)-regolith.properties.minDensity)/(regolith.properties.maxDensity - regolith.properties.minDensity);
          std::cout<<"relative density: "<<relativeDensity<<std::endl;
          correctionFactor = correction_factor(relativeDensity, BOX_DIAMETER/2./probeRadius);
          constexpr double p0 = 1000*100;
          auto expectedResistance = 23.19*p0*std::pow(pressure/p0, 0.56)*std::exp(2.97*relativeDensity);
          auto expectedMeasuredResistance = expectedResistance/correctionFactor;
          std::cout<<"correctionFactor: "<<correctionFactor<<std::endl;
          std::cout<<"expected resistance to be measured: "<<expectedResistance<<std::endl;
        }
    } else if (phase == PENETRATION_PHASE) {
      auto v_0 = probe->getLinearVelocity();
      auto mass = 1./probe->getInvMass();
      auto momentumChange = (btVector3(0., probeVelocity, 0.) - v_0).getY()*mass;
      auto gravityMomentumChange = m_dynamicsWorld->getGravity().getY()*mass*steps_since_last_update*dt;
      auto resistanceForce = (momentumChange)/(steps_since_last_update*dt);
      auto resistance = resistanceForce / (probeRadius*probeRadius*SIMD_PI);
      std::cout<<resistance<<std::endl;
      probe->setLinearVelocity(btVector3(0.,probeVelocity,0.));
      probe->getWorldTransform().setOrigin(probe->getWorldTransform().getOrigin() + btVector3(0., probeVelocity, 0.)*steps_since_last_update*dt);
      auto probeIndex = m_dynamicsWorld->getNumCollisionObjects() - 1;
      btCollisionObject* obj=m_dynamicsWorld->getCollisionObjectArray()[probeIndex];
      btRigidBody* body=btRigidBody::upcast(obj);
      btTransform trans;
      if(body&&body->getMotionState()) {
        body->getMotionState()->getWorldTransform(trans);
      } else {
        trans=obj->getWorldTransform();
      }
    } else if (phase == FINISHED_PHASE) {
    }
    steps_since_last_update = 0;
  }

}

void ConePenetrationTest::addGrains(int layers, bool initial) {
  btTransform startTransform;
  startTransform.setIdentity();
  auto random_engine = std::mt19937{std::random_device{}()};
  auto vertical_vel_distribution = std::uniform_real_distribution<btScalar>{-0.05, 0.05};
  double y = initial?0:BOX_H;
  const double R = initial?(BOX_DIAMETER/2.):(BOX_DIAMETER/3.);
  const double max_y = y+layers*regolith.properties.maxRadius*2;
  while(y<max_y)
  {
    y+=regolith.properties.maxRadius*2;
    double x = -BOX_DIAMETER;
    while(x<BOX_DIAMETER)
    {
      x+=regolith.properties.maxRadius*2;
      double z = -BOX_DIAMETER;
      while(z<BOX_DIAMETER)
      {
        z+=regolith.properties.maxRadius*2;
        if(x*x + z*z < R*R)
        {
          startTransform.setOrigin(btVector3(x, y, z));
          auto body = regolith.createGrain(this, startTransform);
          body->setLinearVelocity(btVector3(vertical_vel_distribution(random_engine), initial?-y:-10, vertical_vel_distribution(random_engine)));
        }
      }
    }
  }
  m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


int ConePenetrationTest::removeGrains() {
  int removed = 0;
  for(auto i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i) {
    auto body = m_dynamicsWorld->getCollisionObjectArray()[i];
    auto pos = body->getWorldTransform().getOrigin();
    if(body->getCollisionShape()->getShapeType()==SPHERE_SHAPE_PROXYTYPE and pos.getY()>BOX_H) {
      ++removed;
      m_dynamicsWorld->removeRigidBody(btRigidBody::upcast(body));
    }
  }
  std::cout<<"REMOVED: "<<removed<<std::endl;
  return removed;
}

void ConePenetrationTest::resetVelocities() {
  for(auto i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i) {
    auto body = m_dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody::upcast(body)->setLinearVelocity(btVector3(0.,0.,0.));
  }
}
  

void ConePenetrationTest::createProbe() {
  btCompoundShape* probeShape = new btCompoundShape();
  btTransform probeTransform;

  btScalar probeH =  1./std::tan(30./360.*SIMD_PI)*probeRadius;
  btConeShape* tipShape = new btConeShape(probeRadius, probeH);
  auto rot = btQuaternion(btVector3(1.,0.,0.), SIMD_PI);
  probeTransform.setRotation(rot);
  probeTransform.setOrigin(btVector3(0.,probeH/2.,0.));
  probeShape->addChildShape(probeTransform, tipShape);

  btScalar rodLength = BOX_H;
  btCollisionShape* rodShape = new btCylinderShape(btVector3{probeRadius,rodLength/2.,probeRadius});
  probeTransform.setIdentity();
  probeTransform.setOrigin(btVector3(0., probeH+rodLength/2., 0.));
  probeShape->addChildShape(probeTransform, rodShape);

  m_collisionShapes.push_back(probeShape);

  probeTransform.setOrigin(btVector3(0., BOX_H, 0.));
  
  auto probeMass = 100.;
  probe = createRigidBody(probeMass, probeTransform, probeShape);
  probe->setLinearVelocity(btVector3{0., probeVelocity, 0.});
  probe->setIgnoreCollisionCheck(pressurePlate, true);
  m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void ConePenetrationTest::createPressurePlate() {
  auto margin = 0.05;
  auto pressurePlateShape = new btCylinderShape{btVector3{BOX_DIAMETER/2. - margin, pressurePlateThickness, BOX_DIAMETER/2. - margin}};
  auto plateTransform = btTransform{};
  plateTransform.setIdentity();
  plateTransform.setOrigin(btVector3(0., BOX_H + pressurePlateThickness + 2*regolith.properties.maxRadius , 0.));
  m_collisionShapes.push_back(pressurePlateShape);
  auto plateMass = pressure*(BOX_DIAMETER/2.1*BOX_DIAMETER/2.1*SIMD_PI)/(-m_dynamicsWorld->getGravity().getY());
  std::cout<<"plate mass: "<<plateMass<<std::endl;
  pressurePlate = createRigidBody(plateMass, plateTransform, pressurePlateShape);
  m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}
}

CommonExampleInterface* CreateFunc(CommonExampleOptions& options)
{
	return new ConePenetrationTest(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(CreateFunc)
