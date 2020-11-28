#include "btBulletDynamicsCommon.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"

#include <iostream>

namespace profiled {

class ProfileZone
{
public:
  const std::string m_name;
  btClock m_clock;
  ProfileZone(const char* name) : m_name(name), m_clock() {
    CProfileManager::Start_Profile(name);
    std::cout<<"Start profile: "<<m_name<<std::endl;
  }
  ~ProfileZone() {
    CProfileManager::Stop_Profile();
    std::cout<<"Stop profile: "<<m_name<<" "<<m_clock.getTimeMilliseconds()<<std::endl;
  }
};

class SolverMt : public btSequentialImpulseConstraintSolverMt
{
  using ParentClass = btSequentialImpulseConstraintSolverMt;

public:
  BT_DECLARE_ALIGNED_ALLOCATOR();
  using ParentClass::ParentClass;

  // for profiling
  virtual btScalar solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies,
                                                 btPersistentManifold** manifoldPtr,
                                                 int numManifolds, btTypedConstraint** constraints,
                                                 int numConstraints,
                                                 const btContactSolverInfo& infoGlobal,
                                                 btIDebugDraw* debugDrawer) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    btScalar ret = ParentClass::solveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr,
                                                             numManifolds, constraints,
                                                             numConstraints, infoGlobal,
                                                             debugDrawer);
    return ret;
  }

  virtual btScalar solveGroupCacheFriendlyIterations(btCollisionObject** bodies, int numBodies,
                                                     btPersistentManifold** manifoldPtr,
                                                     int numManifolds,
                                                     btTypedConstraint** constraints,
                                                     int numConstraints,
                                                     const btContactSolverInfo& infoGlobal,
                                                     btIDebugDraw* debugDrawer) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    btScalar ret = ParentClass::solveGroupCacheFriendlyIterations(bodies, numBodies, manifoldPtr,
                                                                  numManifolds, constraints,
                                                                  numConstraints, infoGlobal,
                                                                  debugDrawer);
    return ret;
  }

  virtual btScalar solveGroupCacheFriendlyFinish(btCollisionObject** bodies, int numBodies,
                                                 const btContactSolverInfo& infoGlobal) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    btScalar ret = ParentClass::solveGroupCacheFriendlyFinish(bodies, numBodies, infoGlobal);
    return ret;
  }

  virtual btScalar solveGroup(btCollisionObject** bodies, int numBodies,
                              btPersistentManifold** manifold, int numManifolds,
                              btTypedConstraint** constraints, int numConstraints,
                              const btContactSolverInfo& info, btIDebugDraw* debugDrawer,
                              btDispatcher* dispatcher) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    btScalar ret = ParentClass::solveGroup(bodies, numBodies, manifold, numManifolds, constraints,
                                           numConstraints, info, debugDrawer, dispatcher);
    return ret;
  }

};

class CollisionDispatcherMt : public btCollisionDispatcherMt
{
  using ParentClass = btCollisionDispatcherMt;
  using ParentClass::ParentClass;

public:

  virtual void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,
                                         const btDispatcherInfo& info,
                                         btDispatcher* dispatcher) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::dispatchAllCollisionPairs(pairCache, info, dispatcher);
  }
};

class AxisSweep: public btAxisSweep3
{
  using ParentClass = btAxisSweep3;
  using ParentClass::ParentClass;
  virtual void calculateOverlappingPairs(btDispatcher* dispatcher)
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::calculateOverlappingPairs(dispatcher);
  }
};

class Dbvt: public btDbvtBroadphase
{
  using ParentClass = btDbvtBroadphase;
  using ParentClass::ParentClass;
  virtual void calculateOverlappingPairs(btDispatcher* dispatcher)
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::calculateOverlappingPairs(dispatcher);
  }
};

ATTRIBUTE_ALIGNED16(class)
World : public btDiscreteDynamicsWorld
{
  using ParentClass = btDiscreteDynamicsWorld;

protected:
  virtual void predictUnconstraintMotion(btScalar timeStep) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::predictUnconstraintMotion(timeStep);
  }
  virtual void createPredictiveContacts(btScalar timeStep) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::createPredictiveContacts(timeStep);
  }
  virtual void integrateTransforms(btScalar timeStep) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::integrateTransforms(timeStep);
  }

public:
  BT_DECLARE_ALIGNED_ALLOCATOR();
  virtual int stepSimulation(btScalar timeStep, int maxSubSteps,
                             btScalar fixedTimeStep) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    return ParentClass::stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
  }
  using ParentClass::ParentClass;

};

ATTRIBUTE_ALIGNED16(class)
WorldMt : public btDiscreteDynamicsWorldMt
{
  using ParentClass = btDiscreteDynamicsWorldMt;

protected:
  virtual void predictUnconstraintMotion(btScalar timeStep) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::predictUnconstraintMotion(timeStep);
  }
  virtual void createPredictiveContacts(btScalar timeStep) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::createPredictiveContacts(timeStep);
  }
  virtual void integrateTransforms(btScalar timeStep) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::integrateTransforms(timeStep);
  }

public:
  BT_DECLARE_ALIGNED_ALLOCATOR();
  virtual int stepSimulation(btScalar timeStep, int maxSubSteps,
                             btScalar fixedTimeStep) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    return ParentClass::stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
  }
  using ParentClass::ParentClass;

};

static void profileBeginCallback(btDynamicsWorld* world, btScalar timeStep)
{
  CProfileManager::Start_Profile("Internal world step");
}

static void profileEndCallback(btDynamicsWorld* world, btScalar timeStep)
{
  CProfileManager::Stop_Profile();
}

} //namespace Profiled
