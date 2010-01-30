/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "shGranularDiscreteDynamicsWorld.h"

//collision detection
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btQuickprof.h"

//rigidbody & constraints
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

//for debug rendering
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "LinearMath/btIDebugDraw.h"


#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btMotionState.h"





shGranularDiscreteDynamicsWorld::shGranularDiscreteDynamicsWorld( btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration )
:btDiscreteDynamicsWorld( dispatcher, pairCache, constraintSolver, collisionConfiguration )
{

}


shGranularDiscreteDynamicsWorld::~shGranularDiscreteDynamicsWorld()
{

}




void shGranularDiscreteDynamicsWorld::addGranularSphere( btSphereShape* centerSphere, btVector3& centerSpherePosition, btScalar childSphereRadius )
{
	// Normal offset vector for each of the child spheres to the main granule sphere
	//   L = left, R = right, U = up, D = down, B = back, F = front
	btVector3 LUB( -NORMVEC_COMP,  NORMVEC_COMP,  NORMVEC_COMP );
	btVector3 LUF( -NORMVEC_COMP,  NORMVEC_COMP, -NORMVEC_COMP );
	btVector3 LDB( -NORMVEC_COMP, -NORMVEC_COMP,  NORMVEC_COMP );
	btVector3 LDF( -NORMVEC_COMP, -NORMVEC_COMP, -NORMVEC_COMP );
	btVector3 RUB(  NORMVEC_COMP,  NORMVEC_COMP,  NORMVEC_COMP );
	btVector3 RUF(  NORMVEC_COMP,  NORMVEC_COMP, -NORMVEC_COMP );
	btVector3 RDB(  NORMVEC_COMP, -NORMVEC_COMP,  NORMVEC_COMP );
	btVector3 RDF(  NORMVEC_COMP, -NORMVEC_COMP, -NORMVEC_COMP );

	btAlignedObjectArray<btVector3> childSphereOffsets;
	childSphereOffsets.push_back( LUB );
	childSphereOffsets.push_back( LUF );
	childSphereOffsets.push_back( LDB );
	childSphereOffsets.push_back( LDF );
	childSphereOffsets.push_back( RUB );
	childSphereOffsets.push_back( RUF );
	childSphereOffsets.push_back( RDB );
	childSphereOffsets.push_back( RDF );
	
	// SET UP THE RIGID BODY FOR OUR MAIN SPHERE, CENTERSPHERE

	// Set up mass and inertia (rigidbody is dynamic if and only if mass is non zero, otherwise static)
	btScalar mass( 1.0 );
	bool isDynamic = ( mass != 0.f );
	btVector3 localInertia( 0, 0, 0 );
	if ( isDynamic )
		centerSphere->calculateLocalInertia( mass, localInertia );

	// Set up the transform for our main sphere, centerSphere
	btTransform sphTransform;
	sphTransform.setIdentity();
	sphTransform.setOrigin( centerSpherePosition );

	// Set up a rigid body for our main sphere, centerSphere
	btDefaultMotionState* myMotionState = new btDefaultMotionState( sphTransform );
	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, centerSphere, localInertia );
	btRigidBody* centerBody = new btRigidBody( rbInfo );
	centerBody->setActivationState(ISLAND_SLEEPING);

	addRigidBody( centerBody );
	
	// SET UP THE SURROUNDING "CHILD" SPHERES

	float centerSphereRadius = centerSphere->getRadius();
	btScalar offsetMagnitude = centerSphereRadius + childSphereRadius;

	int numChildSpheres = childSphereOffsets.size();
	for ( int j = 0; j < numChildSpheres; j++ )
	{
		btVector3 curOffset = childSphereOffsets[j] * offsetMagnitude;
		btVector3 childSpherePosition( centerSpherePosition + curOffset );
		btCollisionShape* sphShape = new btSphereShape( btScalar(1.) );

		// Set up mass and inertia (rigidbody is dynamic if and only if mass is non zero, otherwise static)
		mass = btScalar( 1.0 );
		isDynamic = ( mass != 0.f );
		localInertia = btVector3( 0, 0, 0 );
		if ( isDynamic )
			sphShape->calculateLocalInertia( mass, localInertia );

		// Set up the transform
		sphTransform.setIdentity();
		sphTransform.setOrigin( childSpherePosition );

		// Using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		myMotionState = new btDefaultMotionState( sphTransform );
		btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, sphShape, localInertia );
		btRigidBody* childBody = new btRigidBody( rbInfo );

		childBody->setActivationState(ISLAND_SLEEPING);

		// Set up the constraints to the main sphere
		btVector3 pivA = centerSpherePosition;
		btVector3 pivB = childSpherePosition;
		btPoint2PointConstraint* sphPoint2PointConstraint = new btPoint2PointConstraint( *centerBody, *childBody, pivA, pivB );
		sphPoint2PointConstraint->setDbgDrawSize( btScalar(5.f) );

		addConstraint( sphPoint2PointConstraint, true );
		
	}  // for j

	

}  // addGranularSphere()


