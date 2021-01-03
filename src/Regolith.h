#include "btBulletDynamicsCommon.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include "yaml-cpp/yaml.h"

#include <vector>
#include <string>

namespace regolith {


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

RegolithProperties loadPropertiesFromYaml(const YAML::Node& config);
RegolithProperties loadPropertiesFromFile(std::string filename);

class Regolith {
	public:
	btRigidBody* createGrain(btTransform& transform);
	btRigidBody* createGrain(btTransform& transform, double r, int index);
	btRigidBody* createGrainFromIndex(btTransform& transform, int shape_index, int index);
	const RegolithProperties properties;
	Regolith(RegolithProperties, unsigned int shapes_count = 10);
	std::vector<double> grain_radii;
	std::vector<btRigidBody> grains;
	std::vector<btDefaultMotionState> motion_states;
	private:
	std::vector<btCollisionShape*> collision_shapes;
	std::vector<btScalar> grain_masses;
};
} // namespace regolith
