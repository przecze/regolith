#include "Regolith.h"

#include "utils.h"

#include "btBulletDynamicsCommon.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include "yaml-cpp/yaml.h"

#include <random>
#include <iostream>

RegolithProperties load_properties_from_file(std::string filename) {
	YAML::Node config = YAML::LoadFile(filename);
  RegolithProperties properties;
  properties.maxRadius = config["maxRadius"].as<btScalar>();
  std::cout<<properties.maxRadius<<std::endl;
  properties.minRadius = config["minRadius"].as<btScalar>();
  std::cout<<properties.maxRadius<<std::endl;
  properties.restitution = config["restitution"].as<btScalar>();
  std::cout<<properties.maxRadius<<std::endl;
  properties.friction = config["friction"].as<btScalar>();
  std::cout<<properties.maxRadius<<std::endl;
  properties.rollingFriction = config["rollingFriction"].as<btScalar>();
  std::cout<<properties.maxRadius<<std::endl;
  properties.materialDensity = config["materialDensity"].as<btScalar>();
  std::cout<<properties.maxRadius<<std::endl;
  properties.maxDensity = config["maxDensity"].as<btScalar>();
  std::cout<<properties.maxRadius<<std::endl;
  properties.minDensity = config["minDensity"].as<btScalar>();
  std::cout<<properties.maxRadius<<std::endl;
  return properties;
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
    collisionShapes.push_back(new btSphereShape(radius));
    const auto mass = 4./3. * std::pow(radius,3) * SIMD_PI * properties.materialDensity;
    grainMasses.push_back(mass);
  }
}

btRigidBody* Regolith::createGrain(CommonRigidBodyBase* bodyBase, btTransform& transform) {
  // select shape randomly
  auto random_engine = std::mt19937{std::random_device{}()};
  auto uniform_distribution = std::uniform_int_distribution<int>{0, int(collisionShapes.size())-1};
  int selection = uniform_distribution(random_engine);
  const auto shape = collisionShapes[selection];

  // create and configure body
  auto body = bodyBase->createRigidBody(grainMasses[selection],
                                        transform,
                                        shape,
                                        btVector4(0,0,1,1));
  body->setFriction(properties.friction);
  body->setRestitution(properties.restitution);
  body->setRollingFriction(properties.rollingFriction);
  return body;
}
