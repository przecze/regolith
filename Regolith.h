#include "btBulletDynamicsCommon.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include <vector>

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

class Regolith {
  public:
  btRigidBody* createGrain(CommonRigidBodyBase* base, btTransform& transform);
  const RegolithProperties properties;
  Regolith(RegolithProperties, unsigned int shapesCount = 10);
  private:
  std::vector<btCollisionShape*> collisionShapes;
  std::vector<btScalar> grainMasses;
};
