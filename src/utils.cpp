#include "utils.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include <iostream>
namespace regolith::utils {
  btCompoundShape* BuildTowerCompoundShape(btVector3&& brickFullDimensions,
                                                  unsigned int numRows,
                                                  unsigned int numBricksPerRow,
                                                  bool useConvexHullShape)
  {
      if (numBricksPerRow<3) numBricksPerRow = 3;

      const btVector3 brickHalfDimensions = brickFullDimensions*btScalar(0.5);
      const btScalar h=brickFullDimensions.y(); const btScalar hh= h*0.5f;
      const btScalar l=brickFullDimensions.x(); const btScalar hl= l*0.5f;
      const btScalar z=brickFullDimensions.z(); const btScalar hz= z*0.5f;

      const btScalar perimeter = l*numBricksPerRow;
      const btScalar radius = perimeter/(2.0f*3.1415f) + hz;
      const btScalar elementOffsetAngle = btRadians(360.0f/numBricksPerRow);
      const btScalar floorOffsetAngleBase = elementOffsetAngle * 0.5f;
      btScalar collisionMargin(0.0);

      btCollisionShape* shape = NULL;
      if (!useConvexHullShape) shape = new btBoxShape(brickHalfDimensions);
      else	{
        const btScalar HL = hl + z * btTan(floorOffsetAngleBase);

        btVector3 points[8]={
          btVector3(HL,hh,hz),
          btVector3(-HL,hh,hz),
          btVector3(hl,hh,-hz),
          btVector3(-hl,hh,-hz),

          btVector3(HL,-hh,hz),
          btVector3(-HL,-hh,hz),
          btVector3(hl,-hh,-hz),
          btVector3(-hl,-hh,-hz)
        };
        //btConvexHullShape* chs = new btConvexHullShape(&points.x(),sizeof(points)/sizeof(points[0]));
        // /*
        btConvexHullShape* chs = new btConvexHullShape();
        for (int t=0;t<sizeof(points)/sizeof(points[0]);t++) chs->addPoint(points[t]);
        // */
        shape = chs;
        collisionMargin = 2.0f * shape->getMargin();	// ...so when using the convex hull shape we keep it into considerations (even if decreasing it probably helps)
      }
      if (!shape) return NULL;

      btCompoundShape* csh = new btCompoundShape();

      btTransform T=btTransform::getIdentity();
      T.setOrigin(T.getOrigin()+T.getBasis().getColumn(1)*(hh+collisionMargin*0.5f));
      btTransform T2;
      {
        for (unsigned f=0;f< numRows;f++)	{
          btScalar floorOffsetAngle = (f%2==1 ? floorOffsetAngleBase : 0.0f);
          for (unsigned t=0;t< numBricksPerRow;t++)	{
            T2=T;
            T2.setRotation(btQuaternion(t * elementOffsetAngle+ floorOffsetAngle,0,0));//assignAY( t * elementOffsetAngle+ floorOffsetAngle );
            T2.setOrigin(T2.getOrigin()+T2.getBasis().getColumn(2)*radius);
            // Body Creation:
            csh->addChildShape(T2,shape);
          }
          T.setOrigin(T.getOrigin()+T.getBasis().getColumn(1)*(h+collisionMargin));
        }
      }

      return csh;
  }
} // namespace utils
