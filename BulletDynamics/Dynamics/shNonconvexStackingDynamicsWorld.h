#ifndef BT_NONCONVEXSTACKING_DYNAMICS_WORLD_H
#define BT_NONCONVEXSTACKING_DYNAMICS_WORLD_H

#include "btDiscreteDynamicsWorld.h"


class shNonconvexStackingDynamicsWorld : public btDiscreteDynamicsWorld
{
public:
	shNonconvexStackingDynamicsWorld( btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration );

protected:
	virtual void internalSingleStepSimulation( btScalar timeStep);		// Overwrites parent method
	virtual void synchronizeSingleMotionState( btRigidBody* body );		// Overwrites parent method
	//virtual void integrateTransforms( btScalar timeStep );
	void saveCurrentState( btScalar timeStep );		// saves out each collision object's current transform and velocities
	void revertTransforms();		// set the collision objects' transforms back to their saved positions (but not the velocities!)
	//virtual void calculateSimulationIslands();
	//virtual void solveConstraints(  btContactSolverInfo& solverInfo );

};  // class shNonconvexStackingDynamicsWorld

#endif