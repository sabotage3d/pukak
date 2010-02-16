#ifndef MOLECULAR_DYNAMICS_CONSTRAINT_SOLVER_H
#define MOLECULAR_DYNAMICS_CONSTRAINT_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
class btIDebugDraw;
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"
#include "BulletDynamics/ConstraintSolver/shSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"


#define PI_SQUARED 9.869604401089358618818



///The shNonconvexStackingConstraintSolver is an implementation of the 2003 paper by Guendelman et. al., "Nonconvex Rigid Bodies with Stacking"
class shMolecularDynamicsConstraintSolver : public btConstraintSolver
{
public:
	shMolecularDynamicsConstraintSolver();
	virtual ~shMolecularDynamicsConstraintSolver();
	virtual	void reset();

	virtual btScalar solveGroup( btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc, btDispatcher* dispatcher );

	// Collision constraints
	btScalar solveCollisionConstraints( btPersistentManifold** manifoldPtr, int numManifolds, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc );
	void computeCollisionImpulses();			// Compute the impulses for each solver body
	btScalar computeSoftSphereCollision( btPersistentManifold* manifold, const btContactSolverInfo& infoGlobal );	// Find the deepest intersecting point on the manifold and compute its contact
	void computeVelocitiesForCollisionPair( btRigidBody* rbA, btRigidBody* rbB, btSolverConstraint& solverConstraint );	// Given a pair of rigid bodies and their contact info, compute the local velocities of the contact points and from that the relative velocity of the contact
	void computeImpulseForCollisionPair( btRigidBody* rbA, btRigidBody* rbB, btSolverConstraint& solverConstraint );	// Given a pair of rigid bodies and their contact info, compute the resulting impulse of the contact
	void applyImpulseForCollisionPair( btRigidBody* rbA, btRigidBody* rbB, btSolverConstraint& solverConstraint );		// Apply the impulse for computing the changes in linear and angular velocities for both objects
	void updateVelocitiesForCollisionPair( int objectIdA, int objectIdB, btRigidBody* rbA, btRigidBody* rbB, btScalar timeStep );	// Update the velocities based off the new delta velocities (which were computed from the impulse)

	// Non-contact constraints
	btScalar solveNonContactConstraints( btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc );

	// Collision test results
	bool thereWasACollision();

protected:
	btScalar CollisionHappened;

	btAlignedObjectArray<shSolverBody>	m_tmpSolverBodyPool;
	btConstraintArray m_tmpSolverNonContactConstraintPool;
	btConstraintArray m_tmpSolverContactConstraintPool;
	
	void initSolverBody( shSolverBody* solverBody, btCollisionObject* collisionObject );
	int	getOrInitSolverBody( btCollisionObject& body );

};  // class shMolecularDynamicsConstraintSolver

#endif