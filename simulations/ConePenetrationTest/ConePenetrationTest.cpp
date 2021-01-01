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

#include "profiled.h"
#include "utils.h"
#include "Regolith.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h"

#include "packgen/gen_pack.h"
#include "yaml-cpp/yaml.h"
#include "nlohmann/json.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <random>
#include <algorithm>
#include <cstdlib>

#include <thread>


namespace {

double gColors[4][4] =
  {
		{60. / 256., 186. / 256., 84. / 256., 1},
  	{244. / 256., 194. / 256., 13. / 256., 1},
		{219. / 256., 50. / 256., 54. / 256., 1},
		{72. / 256., 133. / 256., 237. / 256., 1},
};

namespace utils = regolith::utils;

class OverlapReporter : public btOverlappingPairCallback {
  std::vector<std::pair<btBroadphaseProxy*, btBroadphaseProxy*>> pairs;
	btBroadphasePair* addOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) override {
		auto __profile = profiled::ProfileZone(__FUNCTION__);
		pairs.push_back(std::make_pair(proxy0, proxy1));
  }

	void* removeOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, btDispatcher* dispatcher) override {
		auto __profile = profiled::ProfileZone(__FUNCTION__);
		pairs.erase(std::remove_if(pairs.begin(), pairs.end(), [=](auto x) {return (proxy0 == x.first and proxy1 == x.second) or (proxy0 == x.first and proxy1 == x.second); }));
	}

	void removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy0, btDispatcher* dispatcher) override {
		auto __profile = profiled::ProfileZone(__FUNCTION__);
		std::cout<<"removing all pairs for proxy"<<std::endl;
	}

	public:
	const auto& get_current_pairs() { return pairs; }
};

double correction_factor_old(double Dr, double Rd) {
	// Left because values are really small
	//
	// But with -b instead of b in the exponent - which was supposed to work in BC3 case, according to:
	// Jamiolkowski et al, 2003,
	// Evaluation of Relative Density and Shear Strength of Sands from CPT and DMT
	auto a = 9*0.00001*std::pow(Rd, 2.02);
	auto b = -0.565*std::log(Rd)+2.59;
	// TODO: Verify if "100*" should be here
	return a*std::pow(100*Dr, -b);
}

double correction_factor(double Dr, double Rd) {
	// Mayne & Kulhawy 1991
	// As given in Butlanska et al. 2010 (Size effects on a virtual calibration chamber)
	return std::pow((Rd-1)/70, -Dr/2.);
}

struct ConePenetrationTest : public CommonRigidBodyBase
{
	ConePenetrationTest(struct GUIHelperInterface* helper,
	                    regolith::RegolithProperties regolith_properties,
						const YAML::Node& config)
		: CommonRigidBodyBase(helper),
		  regolith(regolith_properties, 10),
		  config(YAML::Clone(config)),
		  BOX_DIAMETER(config["box"]["diameter"].as<double>()),
		  BOX_H(config["box"]["height"].as<double>()),
		  probeVelocity(config["probe"]["velocity"].as<double>()),
		  probeRadius(config["probe"]["radius"].as<double>()),
		  pressure(config["pressure"]["value"].as<double>()),
		  pressurePlateThickness(config["pressure"]["plate_thickness"].as<double>()),
		  update_time(config["simulation"]["big_step"].as<double>()),
		  dt(1./(config["simulation"]["small_steps_per_second"].as<double>())),
      profile_level(utils::try_get(config["simulation"]["profile_level"], 0u))
	{
	}
	btITaskScheduler* m_taskScheduler;

	const double BOX_DIAMETER; // m
	const double BOX_H; // m
	const double probeVelocity; // m/s
	const double probeRadius; // m
	const double pressure; // Pa
	const double pressurePlateThickness; // m
	double dt; // s (not const because is later rescaled)
	double update_time; // s
	const unsigned profile_level;

	virtual ~ConePenetrationTest() {}
	void createEmptyDynamicsWorld() override;
	void initPhysics() override;
	void renderScene() override;
	void stepSimulation(float deltaTime) override;
	void resetCamera()
	{
		float dist = 1.6;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.2, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	btRigidBody* probe;
	btRigidBody* pressurePlate;
	void createProbe();
	void createPressurePlate();

	int removeGrains();
	void addGrains(int layers);
	void addInitialGrains();
	void resetVelocities();

	void stepStabilizationPhase();
	void stepPressurePhase();
	void stepPenetrationPhase(int steps_since_last_update, double velocity_change);
	void reportErrorByY();

	enum {
	  STABILIZATION_PHASE,
	  PRESSURE_PHASE,
	  PENETRATION_PHASE,
	  FINISHED_PHASE
	} phase = STABILIZATION_PHASE;

	double correctionFactor = 0.;


	void rescaleTime(float rescaleFactor) {
	  std::cout<<"Rescaling time by factor "<<rescaleFactor<<std::endl;
	  dt/=rescaleFactor;
	  update_time/=rescaleFactor;
	}

	regolith::Regolith regolith;
	YAML::Node config;
  nlohmann::json profiler_data = [](){
		auto j = nlohmann::json();
		j["data"] = nlohmann::json::array();
		return j;}();
	std::vector<btRigidBody*> grains;
	OverlapReporter overlapReporter = OverlapReporter{};
};

// callback options:
// world->setOverlappingPairUserCallback
// world -> internal tick callback -> iterate over dispatcher manifolds (https://gamedev.stackexchange.com/a/120881/129669)
// gContactAddedCallback https://www.youtube.com/watch?v=YweNArzAHs4, also bullet3/examples/SharedMemory/PhysicsServerCommandProcessor.cpp
// dispatcher -> setNearCallback https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=3997 or other option from this thread

void ConePenetrationTest::createEmptyDynamicsWorld() {
  using namespace profiled;

	if (config["simulation"]["broadphase"].as<std::string>() == "axis") {
		std::cout<<"Using axis sweep broadphase"<<std::endl;
		auto broadphase = create<AxisSweep>(profile_level > 1,
		                                    btVector3(-BOX_DIAMETER, -BOX_H/3., -BOX_DIAMETER),
		                                    btVector3(BOX_DIAMETER, BOX_H, BOX_DIAMETER));
		broadphase->setOverlappingPairUserCallback(&overlapReporter);
		m_broadphase = broadphase;
	} else {
		m_broadphase = create<Dbvt>(profile_level > 1);
	}

	if (config["simulation"]["threads"].as<int>() > 1) {
		std::cout<<"multithread"<<std::endl;
		m_taskScheduler = btCreateDefaultTaskScheduler();
		btSetTaskScheduler(m_taskScheduler);

		btDefaultCollisionConstructionInfo cci;
		cci.m_defaultMaxPersistentManifoldPoolSize = 80000;
		cci.m_defaultMaxCollisionAlgorithmPoolSize = 80000;
		m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);
		m_dispatcher = create<CollisionDispatcherMt>(profile_level > 1, m_collisionConfiguration, 40);

		btConstraintSolverPoolMt* solverPool;
		{
			btConstraintSolver* solvers[BT_MAX_THREAD_COUNT];
			int maxThreadCount = config["simulation"]["threads"].as<int>();
			std::cout<<"Thread count "<<maxThreadCount<<std::endl;
			for (int i = 0; i < maxThreadCount; ++i)
			{
				solvers[i] = create<Solver>(profile_level > 1);
			}
			solverPool = new btConstraintSolverPoolMt(solvers, maxThreadCount);
			m_solver = solverPool;
			btGetTaskScheduler()->setNumThreads(maxThreadCount);
		}
		auto solverMt = create<SolverMt>(profile_level > 1);
		m_dynamicsWorld = create<WorldMt>(profile_level > 1, m_dispatcher, m_broadphase, solverPool,
		                                            solverMt, m_collisionConfiguration);
	} else {
		std::cout<<"single thread"<<std::endl;
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = create<CollisionDispatcher>(profile_level > 1, m_collisionConfiguration);
		btSequentialImpulseConstraintSolver* sol = create<Solver>(profile_level > 1);
		m_solver = sol;
		m_dynamicsWorld = create<World>(profile_level > 1, m_dispatcher, m_broadphase, m_solver,
		                                          m_collisionConfiguration);
	}

	if (profile_level > 0) {
		m_dynamicsWorld->setInternalTickCallback(profileBeginCallback, NULL, true);
		m_dynamicsWorld->setInternalTickCallback(profileEndCallback, NULL, false);
	}

	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
}

void ConePenetrationTest::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	// create ground
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(BOX_DIAMETER),
                                                     btScalar(BOX_H/2),
                                                     btScalar(BOX_DIAMETER)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -BOX_H/2, 0));
	auto ground = createRigidBody(0., groundTransform, groundShape, btVector4(0, 0, 1, 0));
	ground->setFriction(utils::try_get<double>(config["box"]["friction"], 0.));
	ground->setRestitution(utils::try_get<double>(config["box"]["restitution"], 0.));
	ground->setRollingFriction(utils::try_get<double>(config["box"]["rolling_friction"], 0.));

	// create walls as single tower shape
	auto wall_thickness = config["box"]["wall_thickness"].as<double>();
	auto towerShape = utils::BuildTowerCompoundShape(
	                           btVector3{(BOX_DIAMETER+wall_thickness)/(std::sqrt(5 + 2.*std::sqrt(5))),
	                                      BOX_H*2.,
	                                      wall_thickness},
	                           1);
	m_collisionShapes.push_back(towerShape);
	auto tower = createRigidBody(0., groundTransform, towerShape, btVector4(0, 0, 0, 0));
	tower->setFriction(utils::try_get<double>(config["box"]["friction"], 0.));
	tower->setRestitution(utils::try_get<double>(config["box"]["restitution"], 0.));
	tower->setRollingFriction(utils::try_get<double>(config["box"]["rolling_friction"], 0.));

	// add initial grains
	addInitialGrains();
	std::cout<<"Initial grains count: "<<grains.size()<<std::endl;

	// initialize graphics
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	for(auto grain: grains) {
		auto r =dynamic_cast<btSphereShape*>(grain->getCollisionShape())->getRadius();
		auto radius_factor = (regolith.properties.maxRadius - r)/(regolith.properties.maxRadius-regolith.properties.minRadius)*100;
		const double color[4] = {205./256.,194/256.,(100.+radius_factor)/256.,1.};
		m_guiHelper->changeRGBAColor(grain->getUserIndex(), color);
	}
	const double transparent[4] = {0.,0.,0.,0.05};
	m_guiHelper->changeRGBAColor(ground->getUserIndex(), transparent);
	m_guiHelper->changeRGBAColor(tower->getUserIndex(), transparent);
}

void ConePenetrationTest::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

void ConePenetrationTest::stepStabilizationPhase()
{
	int removed = removeGrains();
	resetVelocities();
	if(removed == 0) {
	  phase=PRESSURE_PHASE;
	  //rescaleTime(4.);
	  std::cout<<"Entering PRESSURE_PHASE"<<std::endl;
	  createPressurePlate();
	}
}

void ConePenetrationTest::reportErrorByY()
{
	auto h = pressurePlate->getWorldTransform().getOrigin().getY()-pressurePlateThickness/2;
	auto buckets = 10u;
	auto error_per_bucket = std::vector<double>(buckets, 0.);
	auto pairs_per_bucket = std::vector<unsigned>(buckets, 0);

	auto manifolds_count = m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for(auto i = 0u; i<manifolds_count; ++i) {
		auto manifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		if (not manifold->getNumContacts()) {
			continue;
		}
		auto *objA = manifold->getBody0();
		auto *objB = manifold->getBody1();
		auto r1 =static_cast<const btSphereShape*>(objA->getCollisionShape())->getRadius();
		auto r2 =static_cast<const btSphereShape*>(objB->getCollisionShape())->getRadius();
		auto y1 = objA->getWorldTransform().getOrigin().getY();
		auto y2 = objB->getWorldTransform().getOrigin().getY();
		auto bucket = int(10*(y1+y2)/2./h);
		if (bucket < buckets) {
			pairs_per_bucket[bucket]++;
			error_per_bucket[bucket]+=manifold->getContactPoint(0).m_distance1;
		}

	}

	//for(auto overlap_pair : overlapReporter.get_current_pairs()){
	//	auto proxy1 = overlap_pair.first;
	//	auto proxy2 = overlap_pair.second;
	//	auto position1 = (proxy1->m_aabbMax + proxy1->m_aabbMin)/2.;
	//	auto r1 = (proxy1->m_aabbMax - position1).length()/1.41;
	//	auto position2 = (proxy2->m_aabbMax + proxy2->m_aabbMin)/2.;
	//	auto r2 = (proxy2->m_aabbMax - position2).length()/1.41;
	//	auto distance = (position2 - position1).length();
	//	auto error = (r1 + r2) - distance;

	//	if (bucket < buckets) {
	//		pairs_per_bucket[bucket]++;
	//		error_per_bucket[bucket]+=error;
	//	}
	//}

	for(int i = 0; i<buckets; ++i) {
		std::cout<<"bucket: "<<i<<" total error: "<<error_per_bucket[i]<<" pairs: "<< pairs_per_bucket[i] << " error per pair: " << error_per_bucket[i]/pairs_per_bucket[i]<<std::endl;
	}
}

void ConePenetrationTest::stepPressurePhase()
{
	removeGrains();
	//reportErrorByY();
	auto plate_y = pressurePlate->getWorldTransform().getOrigin().getY();
	auto plate_v = pressurePlate->getLinearVelocity().getY();
	std::cout<<"plate y: "<<plate_y<<" v: "<<plate_v<<std::endl;

	if(abs(plate_v)<0.01) {
		std::cout<<"Entering PENETRATION_PHASE"<<std::endl;
		phase = PENETRATION_PHASE;
		createProbe();
		auto h = pressurePlate->getWorldTransform().getOrigin().getY()-pressurePlateThickness/2;
		auto V = h*BOX_DIAMETER/2.*BOX_DIAMETER/2.*SIMD_PI;
		double grains_mass = 0.;
		double grains_volume = 0.;
		auto buckets = 10u;

		for(auto grain: grains) {
			auto grain_mass = 1./grain->getInvMass();
			grains_mass += grain_mass;

			auto r =dynamic_cast<btSphereShape*>(grain->getCollisionShape())->getRadius();
			auto grain_volume = 4./3.*SIMD_PI*r*r*r;

			grains_volume += grain_volume;
		}

		std::cout<<"void ratio: "<<(V-grains_volume)/grains_volume<<std::endl;
		std::cout<<"\% of volume used: "<<grains_volume/V*100.<<std::endl;

		auto relativeDensity = ((grains_mass/V)-regolith.properties.minDensity)/(regolith.properties.maxDensity - regolith.properties.minDensity);
		std::cout<<"relative density: "<<relativeDensity<<std::endl;
		auto Rd = BOX_DIAMETER/2./probeRadius;
		correctionFactor = correction_factor(relativeDensity, Rd);
		constexpr double p0 = 1000*100;
		auto expectedResistance = 23.19*p0*std::pow(pressure/p0, 0.56)*std::exp(2.97*relativeDensity);
		auto expectedMeasuredResistance = expectedResistance/correctionFactor;
		std::cout<<"correctionFactor: "<<correctionFactor<<std::endl;
		//std::cout<<"correctionFactorold: "<<correction_factor_old(relativeDensity, Rd)<<std::endl;
		std::cout<<"expected resistance to be measured: "<<expectedResistance<<std::endl;
		update_time = config["simulation"]["penetration"]["big_step"].as<double>();
	}
}

void ConePenetrationTest::stepPenetrationPhase(int steps_since_last_update, double velocity_change) {
	static double initial_y = 1000.f;
	static double total_time = 0.f;
	auto y = probe->getWorldTransform().getOrigin().getY();

	// sanity check that the average speed is similar to desired probeVelocity
	if(initial_y == 1000.) {
		initial_y = y; 
		total_time -= steps_since_last_update*dt;
	}
	total_time += steps_since_last_update*dt;
	std::cout<<"distance:"<<(y-initial_y)<<std::endl;
	std::cout<<"total_time:"<<total_time<<std::endl;
	std::cout<<"average speed:"<<(y-initial_y)/total_time<<std::endl;

	// calculate and report resistance based on total velocity change accumulated since last update
	auto mass = 1./probe->getInvMass();
	auto momentumChange = velocity_change*mass;
	auto resistanceForce = (momentumChange)/(steps_since_last_update*dt);
	auto resistance = resistanceForce / (probeRadius*probeRadius*SIMD_PI);
	std::cout<<"y: "<<y<<" resistance: "<<resistance<<std::endl;
  if (y<BOX_H/5.) {
    phase = FINISHED_PHASE;
  }
}

void ConePenetrationTest::stepSimulation(float deltaTime)
{
	using namespace std::chrono;
	static auto last = steady_clock::now();
	static int steps_since_last_update = 0;
	static int grains_count = 0;
	static double probe_velocity_change = 0;
	{
		// note: stepSimulation doesn't return the actual steps performed, but
		// deltaTime/dt therefore we disable clamping and ensure only one step is made.
		m_dynamicsWorld->stepSimulation(deltaTime, 1, dt);
		steps_since_last_update += 1;
	}

	// in penetration phase there are things we need to do every step
	if (phase == PENETRATION_PHASE) {
		probe_velocity_change += probe->getLinearVelocity().getY()-probeVelocity;
		probe->setLinearVelocity(btVector3(0., 1.03*probeVelocity, 0.));
	}

	if(steps_since_last_update*dt > update_time) {
		if (profile_level > 0) {
			simprof::Manager::start("Additional logic");
		}
		auto current = steady_clock::now();
		auto elapsed = current - last;
		last = current;
		std::cout<<"last step took: "<<std::chrono::duration_cast<milliseconds>(elapsed).count()<<" ms"<<std::endl;
		if (phase == STABILIZATION_PHASE) {
			stepStabilizationPhase();
		} else if (phase == PRESSURE_PHASE) {
  		stepPressurePhase();
		} else if (phase == PENETRATION_PHASE) {
			stepPenetrationPhase(steps_since_last_update, probe_velocity_change);
			probe_velocity_change = 0;
		} else if (phase == FINISHED_PHASE) {
  		std::exit(0);
		}
		steps_since_last_update = 0;
		if (profile_level > 0) {
			simprof::Manager::stop();
			auto profiler_dump = simprof::Manager::dump_json();
  		profiler_dump["phase"] = phase;
			//std::cout << std::setw(4) << profiler_dump << std::endl;
  		profiler_data["data"].push_back(profiler_dump);
			std::ofstream("profiler_data.json") << profiler_data << std::endl;
		}
	}

}

void ConePenetrationTest::addInitialGrains() {

  auto sizes_count = regolith.grain_radii.size();
  double p[sizes_count];
  std::fill_n(p, sizes_count, 1./sizes_count);
  double* r = &regolith.grain_radii[0];
	PG::NG* ng = new PG::GeneralNG(r,
                                 p,
                                 sizes_count);

	//PG::NG* ng = new PG::UniformNG(0.9*regolith.minRadius,
  //                               0.9*regolith.maxRadius);
	PG::Grid3d dom;
	PG::Container* container = new PG::Cylinder({0.0, 0.0, 0.0},
                                              {0.0, BOX_H, 0.0},
                                              BOX_DIAMETER/2);

	PG::SpherePack* pack = new PG::SpherePack();
	PG::SpherePackStat result = PG::GenerateSpherePack(container, ng, &dom, pack);

	for(auto s: pack->s) {
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(s.x, s.y, s.z));
		auto grain = regolith.createGrain(this, transform, s.r);
		grains.push_back(grain);
	}
}

int ConePenetrationTest::removeGrains() {
	const double transparent[4] = {0.,0.,0.,0.};
	int removed = 0;
	for(auto it = grains.begin(); it != grains.end();) {
		auto pos = (*it)->getWorldTransform().getOrigin();
		if(pos.getY() > BOX_H) {
			++removed;
			m_guiHelper->changeRGBAColor((*it)->getUserIndex(), transparent);
			m_dynamicsWorld->removeRigidBody(*it);
			it = grains.erase(it);
		} else {
			++it;
		}
	}
	std::cout<<"REMOVED: "<<removed<<std::endl;
	return removed;
}

void ConePenetrationTest::resetVelocities() {
	for(auto grain: grains) {
		grain->setLinearVelocity(btVector3(0.,0.,0.));
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

	auto probe_initial_y = pressurePlate->getWorldTransform().getOrigin().getY() + pressurePlateThickness +0.1;
	probeTransform.setOrigin(btVector3(0., probe_initial_y, 0.));
	
	auto probeMass = config["probe"]["mass"].as<double>();
	probe = createRigidBody(probeMass, probeTransform, probeShape);
	probe->setLinearVelocity(btVector3{0., probeVelocity, 0.});
	probe->setIgnoreCollisionCheck(pressurePlate, true);
	// exclude probe from gravity
	probe->setGravity(btVector3{0.,0.,0.});

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	m_guiHelper->changeRGBAColor(probe->getUserIndex(), gColors[2]);
}

void ConePenetrationTest::createPressurePlate() {
	auto margin = 0.05;
	auto pressurePlateShape = new btCylinderShape{btVector3{BOX_DIAMETER/2. - margin, pressurePlateThickness, BOX_DIAMETER/2. - margin}};
	auto plateTransform = btTransform{};
	plateTransform.setIdentity();
	plateTransform.setOrigin(btVector3(0., BOX_H + pressurePlateThickness/2. + 2*regolith.properties.maxRadius , 0.));
	m_collisionShapes.push_back(pressurePlateShape);
	auto plateMass = pressure*(BOX_DIAMETER/2.1*BOX_DIAMETER/2.1*SIMD_PI)/(-m_dynamicsWorld->getGravity().getY());
	std::cout<<"plate mass: "<<plateMass<<std::endl;
	pressurePlate = createRigidBody(plateMass, plateTransform, pressurePlateShape);
	pressurePlate->setFriction(utils::try_get<double>(config["box"]["friction"], 0.));
	pressurePlate->setRestitution(utils::try_get<double>(config["box"]["restitution"], 0.));
	pressurePlate->setRollingFriction(utils::try_get<double>(config["box"]["rolling_friction"], 0.));
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	const double transparent[4] = {0.,0.,0.,1.};

	m_guiHelper->changeRGBAColor(pressurePlate->getUserIndex(), gColors[3]);
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

	return new ConePenetrationTest(options.m_guiHelper, properties, config);
}

B3_STANDALONE_EXAMPLE(CreateFunc)
