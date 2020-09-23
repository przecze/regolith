#include "Regolith.h"

#include "utils.h"

#include "btBulletDynamicsCommon.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include "yaml-cpp/yaml.h"

#include <random>
#include <iostream>

RegolithProperties load_properties_from_yaml(const YAML::Node& config) {
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

RegolithProperties load_properties_from_file(std::string filename) {
  YAML::Node config = YAML::LoadFile(filename);
  return load_properties_from_yaml(config);
}


Regolith::Regolith(RegolithProperties properties, unsigned int shapesCount):
  properties(properties),
  collisionShapes{},
  grainMasses{} {
  for(int i = 0; i < shapesCount; ++i) {
    assert(shapesCount > 1);
    auto radius = utils::sizeInverseDistribution(i*1./(shapesCount-1),
                                                 properties.minRadius,
                                                 properties.maxRadius);
    grainRadii.push_back(radius);
    collisionShapes.push_back(new btSphereShape(radius));
    const auto mass = 4./3. * std::pow(radius,3) * SIMD_PI * properties.materialDensity;
    grainMasses.push_back(mass);
  }
}

btRigidBody* Regolith::createGrainFromIndex(CommonRigidBodyBase* bodyBase,
                                            btTransform& transform,
                                            int index) {
  const auto shape = collisionShapes[index];

  // create and configure body
  auto body = bodyBase->createRigidBody(grainMasses[index],
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
  // select shape randomly
  auto random_engine = std::mt19937{std::random_device{}()};
  auto uniform_distribution = std::uniform_int_distribution<int>{0, int(collisionShapes.size())-1};
  int selection = uniform_distribution(random_engine);
  return createGrainFromIndex(bodyBase, transform, selection);
}

btRigidBody* Regolith::createGrain(CommonRigidBodyBase* bodyBase,
                                   btTransform& transform) {
  // select shape randomly
  auto random_engine = std::mt19937{std::random_device{}()};
  auto uniform_distribution = std::uniform_int_distribution<int>{0, int(collisionShapes.size())-1};
  int selection = uniform_distribution(random_engine);
  return createGrainFromIndex(bodyBase, transform, selection);
}
