#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

#include "yaml-cpp/yaml.h"

namespace regolith::utils {
  btCompoundShape* BuildTowerCompoundShape(btVector3&& brickFullDimensions=
                                                                btVector3(4.0,3.0,2.0),
                                                  unsigned int numRows=10,
                                                  unsigned int numBricksPerRow=10,
                                                  bool useConvexHullShape=true);
  static double sizeInverseDistribution(double p, double minRadius, double maxRadius) {
    assert(p <=1.);
    assert(p >=0.);
    return maxRadius * std::exp( std::log(minRadius / maxRadius) * p);
  }

  template<typename T>
  T try_get(const YAML::Node& yaml_node, T default_value)
  {
    if (yaml_node) {
      return yaml_node.as<T>();
    }
    return default_value;
  }


} // namespace regolith::utils
