#include "Regolith.h"

#include "utils.h"

#include "btBulletDynamicsCommon.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include "yaml-cpp/yaml.h"

#include <random>
#include <iostream>
#include <stdexcept>

namespace regolith {
RegolithProperties loadPropertiesFromYaml(const YAML::Node& config) {
	RegolithProperties properties;
	const auto units_per_m = utils::try_get(config["units_per_m"], 1.);
	properties.maxRadius = config["maxRadius"].as<btScalar>()*units_per_m;
	properties.minRadius = config["minRadius"].as<btScalar>()*units_per_m;
	properties.restitution = config["restitution"].as<btScalar>();
	properties.friction = config["friction"].as<btScalar>();
	properties.rollingFriction = config["rollingFriction"].as<btScalar>();
	const auto units_per_m3 = std::pow(units_per_m, 3);
	properties.materialDensity = config["materialDensity"].as<btScalar>()/units_per_m3;
	properties.maxDensity = config["maxDensity"].as<btScalar>()/units_per_m3;
	properties.minDensity = config["minDensity"].as<btScalar>()/units_per_m3;
	return properties;
}

RegolithProperties loadPropertiesFromFile(std::string filename) {
	YAML::Node config = YAML::LoadFile(filename);
	return loadPropertiesFromYaml(config);
}


Regolith::Regolith(RegolithProperties properties, unsigned int shapesCount):
	properties(properties),
	collision_shapes{},
	grain_masses{} {
	for(int i = 0; i < shapesCount; ++i) {
	  assert(shapesCount > 1);
	  auto radius = utils::sizeInverseDistribution(i*1./(shapesCount-1),
	                                               properties.minRadius,
	                                               properties.maxRadius);
	  grain_radii.push_back(radius);
	  collision_shapes.push_back(new btSphereShape(radius));
	  const auto mass = 4./3. * std::pow(radius,3) * SIMD_PI * properties.materialDensity;
	  grain_masses.push_back(mass);
	}
}

btRigidBody* Regolith::createGrainFromIndex(btTransform& transform, int shape_index, int index) {
	const auto shape = collision_shapes[shape_index];
	const auto mass = grain_masses[shape_index];

	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);

	motion_states[index] = btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, &motion_states[index], shape, inertia);
	grains[index] = btRigidBody(cInfo);
	auto& body = grains[index];

	body.setUserIndex(-1);
	body.setFriction(properties.friction);
	body.setRestitution(properties.restitution);
	body.setRollingFriction(properties.rollingFriction);
	body.setAngularFactor(0.);
	return &body;
}

btRigidBody* Regolith::createGrain(btTransform& transform, double r, int index) {
	for(int i = 0; i < grain_radii.size(); ++i) {
		double known_r = grain_radii[i];
		if(abs(r - known_r) < (0.0001 * properties.minRadius)) {
			return createGrainFromIndex(transform, i, index);
		}
	}
	throw std::invalid_argument("Value of r not found in known regolith collision shapes");
}

btRigidBody* Regolith::createGrain(btTransform& transform) {
	// select shape randomly
	auto random_engine = std::mt19937{std::random_device{}()};
	auto uniform_distribution = std::uniform_int_distribution<int>{0, int(collision_shapes.size())-1};
	int selection = uniform_distribution(random_engine);
	return createGrainFromIndex(transform, selection, 0);
}
} // namespace regolith
