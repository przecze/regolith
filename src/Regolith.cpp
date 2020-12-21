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
	properties.maxRadius = config["maxRadius"].as<btScalar>();
	properties.minRadius = config["minRadius"].as<btScalar>();
	properties.restitution = config["restitution"].as<btScalar>();
	properties.friction = config["friction"].as<btScalar>();
	properties.rollingFriction = config["rollingFriction"].as<btScalar>();
	properties.materialDensity = config["materialDensity"].as<btScalar>();
	properties.maxDensity = config["maxDensity"].as<btScalar>();
	properties.minDensity = config["minDensity"].as<btScalar>();
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

btRigidBody* Regolith::createGrainFromIndex(CommonRigidBodyBase* bodyBase,
	                                          btTransform& transform,
	                                          int index) {
	const auto shape = collision_shapes[index];

	// create and configure body
	auto body = bodyBase->createRigidBody(grain_masses[index],
	                                      transform,
	                                      shape,
	                                      btVector4(0,0,1,1));
	body->setFriction(properties.friction);
	body->setRestitution(properties.restitution);
	body->setRollingFriction(properties.rollingFriction);
	return body;
}

btRigidBody* Regolith::createGrain(CommonRigidBodyBase* bodyBase,
	                                 btTransform& transform,
	                                 double r) {
	for(int i = 0; i < grain_radii.size(); ++i) {
		double known_r = grain_radii[i];
		if(abs(r - known_r) < (0.0001 * properties.minRadius)) {
			return createGrainFromIndex(bodyBase, transform, i);
		}
	}
	throw std::invalid_argument("Value of r not found in known regolith collision shapes");
}

btRigidBody* Regolith::createGrain(CommonRigidBodyBase* body_base,
	                                 btTransform& transform) {
	// select shape randomly
	auto random_engine = std::mt19937{std::random_device{}()};
	auto uniform_distribution = std::uniform_int_distribution<int>{0, int(collision_shapes.size())-1};
	int selection = uniform_distribution(random_engine);
	return createGrainFromIndex(body_base, transform, selection);
}
} // namespace regolith
