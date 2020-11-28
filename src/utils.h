#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

namespace utils {
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

} // namespace utils
