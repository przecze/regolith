#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"

CommonExampleInterface* CreateFunc(struct CommonExampleOptions& options);

int main(int argc, char* argv[])
{
	DummyGUIHelper noGfx;

	CommonExampleOptions options(&noGfx);
	auto example = CreateFunc(options);

	example->initPhysics();
  while(true) {
    example->stepSimulation(1.f / 60.f);
  }
	example->exitPhysics();

	delete example;

	return 0;
}

