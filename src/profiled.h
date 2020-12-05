#include "btBulletDynamicsCommon.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"

#include <iostream>

namespace profiled {

template <class T, class ...Args>
typename T::ParentClass* create(bool profiled, Args... args) {
  if(profiled) {
    return new T(args...);
  }
  return new typename T::ParentClass(args...);
}

class ProfileZone
{
public:
  ProfileZone(const char* name)
  {
    CProfileManager::Start_Profile(name);
  }
  ~ProfileZone()
  {
    CProfileManager::Stop_Profile();
  }
};

class Solver : public btSequentialImpulseConstraintSolver
{

public:
  using ParentClass = btSequentialImpulseConstraintSolver;
  BT_DECLARE_ALIGNED_ALLOCATOR();
  using ParentClass::ParentClass;

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

class SolverMt : public btSequentialImpulseConstraintSolverMt
{

public:
  using ParentClass = btSequentialImpulseConstraintSolverMt;
  BT_DECLARE_ALIGNED_ALLOCATOR();
  using ParentClass::ParentClass;

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

class CollisionDispatcher : public btCollisionDispatcher
{

public:

  using ParentClass = btCollisionDispatcher;
  using ParentClass::ParentClass;

  virtual void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,
                                         const btDispatcherInfo& info,
                                         btDispatcher* dispatcher) BT_OVERRIDE
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::dispatchAllCollisionPairs(pairCache, info, dispatcher);
  }
};

class CollisionDispatcherMt : public btCollisionDispatcherMt
{

public:
  using ParentClass = btCollisionDispatcherMt;
  using ParentClass::ParentClass;

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
  virtual void calculateOverlappingPairs(btDispatcher* dispatcher)
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::calculateOverlappingPairs(dispatcher);
  }
  public:
  using ParentClass = btAxisSweep3;
  using ParentClass::ParentClass;
};

class Dbvt: public btDbvtBroadphase
{
  virtual void calculateOverlappingPairs(btDispatcher* dispatcher)
  {
    auto __profile = ProfileZone(__FUNCTION__);
    ParentClass::calculateOverlappingPairs(dispatcher);
  }
  public:
  using ParentClass = btDbvtBroadphase;
  using ParentClass::ParentClass;
};

ATTRIBUTE_ALIGNED16(class)
World : public btDiscreteDynamicsWorld
{

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
  using ParentClass = btDiscreteDynamicsWorld;
  using ParentClass::ParentClass;
  BT_DECLARE_ALIGNED_ALLOCATOR();
  virtual int stepSimulation(btScalar timeStep, int maxSubSteps,
                             btScalar fixedTimeStep) BT_OVERRIDE
  {
    return ParentClass::stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
  }

};

ATTRIBUTE_ALIGNED16(class)
WorldMt : public btDiscreteDynamicsWorldMt
{

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
  using ParentClass = btDiscreteDynamicsWorldMt;
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
