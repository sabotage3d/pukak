#include "shNonconvexStackingDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "LinearMath/btQuickprof.h"
#include "shNonconvexStackingConstraintSolver.h"

#include <stdio.h>



shNonconvexStackingDynamicsWorld::shNonconvexStackingDynamicsWorld( btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration )
:btDiscreteDynamicsWorld( dispatcher, pairCache, constraintSolver, collisionConfiguration )
{
}  // constructor




// internalSingleStepSimulation()
//     Overrides method from btDiscreteDynamicsWorld
void shNonconvexStackingDynamicsWorld::internalSingleStepSimulation( btScalar timeStep ) {

	// ******************* //
	// COLLISION DETECTION //
	// ******************* //
printf( "\n\n" );
	// Save out the current positions and velocity so we can revert back after collision testing
	int count = 0;
	do {
		saveCurrentState( timeStep );
		//   Contacts are treated with the correct epsilons of the colliding objects
		//   Process:
		// 1. Update the positions (based on current velocity and forces)
		{
			// Solve for the new position integrating the velocity and forces
			//   This uses x' = x + dt(v + dt(F)), where dt = delta t, or the time step amount
			predictUnconstraintMotion( timeStep );		// Apply gravity, predict motion (this updates the transforms)
			integrateTransforms( timeStep );			// Updates the positions (transform matrix) based on the current velocities
		}
		
		// 2. Perform the collision detection
		//      Use the colliding objects' epsilons
		{
			performDiscreteCollisionDetection();	// defined in parent class btCollisionWorld
			calculateSimulationIslands();
		}

		// 3. Solve for impulses where there are collisions
		//      In solving constraints, velocities are updated (changes in velocity are computed)
		{
			// calculate impulses then calculate the changes (delta) in the linear and angular velocities
			solveConstraints( getSolverInfo() );
		}
		
		// 4. Velocities updated (this happened above in solverConstraints(), where impulses were applied to velocities)
		
		// 5. Revert bodies back to their original positions
		//	 They now have the updated velocities
		{
			revertTransforms();		// All collision objects back to their original positions (x) based on the saved states
			//integrateTransforms( timeStep );
		}
		count++;
	// 6. If any collisions were detected, repeat with v = v'
	} while( count < 50 );	//((shNonconvexStackingConstraintSolver*)m_constraintSolver)->thereWasACollision() );	// m_constraintSolver is a class variable defined in btDiscreteDynamicsWorld.h
	
	predictUnconstraintMotion( timeStep );		// Apply gravity, predict motion (this updates the transforms)
	integrateTransforms( timeStep );			// Updates the positions (transform matrix) based on the current velocities
	//predictUnconstraintMotion( timeStep );		// Apply gravity, predict motion (this updates the transforms)

	// ALL VELOCITIES ARE NOW UPDATED, THOUGH THE OBJECTS ARE STILL IN THEIR ORIGINAL POSITIONS

	// ****************** //
	// CONTACT RESOLUTION //
	// ****************** //
	//   This applies all current forces and the updated velocity
	//   Contacts are treated with epsilon = 0
	//     1. Update the positions with the new velocity
	//     2. Perform the collision detection
	//          Use epsilon = 0 (which is what makes it contact detection)


	// ***************************************************************** //
	// UPDATE POSITIONS (apply forces and updated velocity to positions) //
	// ***************************************************************** //

}  // internalSingleStepSimulation()




/*// integrateTransforms()
//     Overrides method from btDiscreteDynamicsWorld
void shNonconvexStackingDynamicsWorld::integrateTransforms(btScalar timeStep)
{
	BT_PROFILE("integrateTransforms");
	btTransform predictedTrans;
	
	// can possibly use the following variables from btCollisionObject.h for the updated transforms:
	//   btTransform m_interpolationWorldTransform;
	//   btVector3 m_interpolationLinearVelocity;
	//   btVector3 m_interpolationAngularVelocity;
	
}  // integrateTransforms()*/




void shNonconvexStackingDynamicsWorld::synchronizeSingleMotionState( btRigidBody* body )
{
	btAssert(body);

	if (body->getMotionState() && !body->isStaticOrKinematicObject())
	{
		//we need to call the update at least once, even for sleeping objects
		//otherwise the 'graphics' transform never updates properly
		///@todo: add 'dirty' flag
		//if (body->getActivationState() != ISLAND_SLEEPING)
		{
			btTransform interpolatedTransform = body->getWorldTransform();
			//btTransformUtil::integrateTransform(body->getInterpolationWorldTransform(),
			//	body->getInterpolationLinearVelocity(),body->getInterpolationAngularVelocity(),m_localTime*body->getHitFraction(),interpolatedTransform);
			body->getMotionState()->setWorldTransform( interpolatedTransform );
		}
	}
}  // synchronizeSingleMotionState()




void shNonconvexStackingDynamicsWorld::saveCurrentState( btScalar timeStep )
{
	// Take each rigid body and save its current position and velocity
	btCollisionObjectArray& collisionObjects = getCollisionWorld()->getCollisionObjectArray();		// DOES THIS GIVE US EVERY SINGLE OBJECT, OR JUST THE ONES CURRENTLY COLLIDING
	int numCollisionObjects = getCollisionWorld()->getNumCollisionObjects();
	for ( int i = 0; i < numCollisionObjects; i++ )
	{
		btCollisionObject* colObj = collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if ( body )
		{
			// Saves out the world transforms and velocities of each collision (rigid body) object
			//   (btRigidBody saves them into m_interpolationXX variables, and shRigidBody does that as well as saves them into m_savedXX variables)
			body->saveKinematicState( timeStep );

		}  // if

	}  // for i
	
/*
	// Access each collision pair in the array of manifolds (a manifold is the set of colliding points for a colliding pair)
	// Then take each rigid body in each pair and save it's current position and velocity
	btDispatcher* dispatcher = getCollisionWorld()->getDispatcher();
	btPersistentManifold** manifoldArray = dispatcher->getInternalManifoldPointer();
	btPersistentManifold* manifold = 0;
	int maxNumManifolds = dispatcher->getNumManifolds();
	for ( int i = 0; i < maxNumManifolds; i++ ) {
		
		manifold = manifoldArray[i];
		btCollisionObject* colObj0=0,*colObj1=0;
		colObj0 = (btCollisionObject*)manifold->getBody0();
		colObj1 = (btCollisionObject*)manifold->getBody1();
		btRigidBody* rb0 = btRigidBody::upcast(colObj0);
		btRigidBody* rb1 = btRigidBody::upcast(colObj1);
		if ( rb0 )
			// Saves out the world transforms and velocities of each collision (rigid body) object
			//   (btRigidBody saves them into m_interpolationXX variables, and shRigidBody does that as well as saves them into m_savedXX variables)
			rb0->saveKinematicState( timeStep );
		if ( rb1 )
			// Saves out the world transforms and velocities of each collision (rigid body) object
			//   (btRigidBody saves them into m_interpolationXX variables, and shRigidBody does that as well as saves them into m_savedXX variables)
			rb1->saveKinematicState( timeStep );

	}  // for i
*/
}  // saveCurrentState()




void shNonconvexStackingDynamicsWorld::revertTransforms()
{
	// Take each rigid body and restore its world transform
	//btCollisionObjectArray& collisionObjects = getCollisionWorld()->getCollisionObjectArray();
	//int numCollisionObjects = getCollisionWorld()->getNumCollisionObjects();
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
	//for ( int i = 0; i < numCollisionObjects; i++ )
	//{
		//btCollisionObject* colObj = collisionObjects[i];
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if ( body )
		{
			//body->resetWorldTransform();		// resetWorldTransform() is in btCollisionObject.h
			body->revertKinematicState();
			body->proceedToTransform( body->getWorldTransform() );		// update the object's world transformation matrix and inertia tensor

		}  // if

	}  // for i

	/*// Access each collision pair in the array of manifolds (a manifold is the set of colliding points for a colliding pair)
	// Then take each rigid body and restore its world transform
	btDispatcher* dispatcher = getCollisionWorld()->getDispatcher();
	btPersistentManifold** manifoldArray = dispatcher->getInternalManifoldPointer();
	btPersistentManifold* manifold = 0;
	int maxNumManifolds = dispatcher->getNumManifolds();
	for ( int i = 0; i < maxNumManifolds; i++ ) {
		
		manifold = manifoldArray[i];
		btCollisionObject* colObj0=0,*colObj1=0;
		colObj0 = (btCollisionObject*)manifold->getBody0();
		colObj1 = (btCollisionObject*)manifold->getBody1();
		btRigidBody* rb0 = btRigidBody::upcast(colObj0);
		btRigidBody* rb1 = btRigidBody::upcast(colObj1);
		if ( rb0 )
		{
			rb0->revertKinematicState();
			rb0->proceedToTransform( rb0->getWorldTransform() );		// update the object's world transformation matrix and inertia tensor

		}  // if
		if ( rb1 )
		{
			rb1->revertKinematicState();
			rb1->proceedToTransform( rb0->getWorldTransform() );		// update the object's world transformation matrix and inertia tensor

		}  // if

	}  // for i*/

}  // revertTransforms()




