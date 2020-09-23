#include "btBulletDynamicsCommon.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include "yaml-cpp/yaml.h"

#include <vector>
#include <string>

struct RegolithProperties {
  btScalar maxRadius;
  btScalar minRadius;
  btScalar restitution;
  btScalar friction;
  btScalar rollingFriction;
  btScalar materialDensity;
  btScalar maxDensity;
  btScalar minDensity;
};

RegolithProperties load_properties_from_yaml(const YAML::Node& config);
RegolithProperties load_properties_from_file(std::string filename);

class Regolith {
  public:
  btRigidBody* createGrain(CommonRigidBodyBase* base, btTransform& transform);
  btRigidBody* createGrain(CommonRigidBodyBase* base, btTransform& transform, double r);
  btRigidBody* createGrainFromIndex(CommonRigidBodyBase* base,
                                    btTransform& transform,
                                    int index);
  const RegolithProperties properties;
  Regolith(RegolithProperties, unsigned int shapesCount = 10);
  std::vector<double> grainRadii;
  private:
  std::vector<btCollisionShape*> collisionShapes;
  std::vector<btScalar> grainMasses;
};
