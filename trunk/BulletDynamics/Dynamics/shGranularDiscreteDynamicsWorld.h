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


#ifndef BT_GRANULAR_DISCRETE_DYNAMICS_WORLD_H
#define BT_GRANULAR_DISCRETE_DYNAMICS_WORLD_H

#include "btDiscreteDynamicsWorld.h"

class btDispatcher;
class btOverlappingPairCache;
class btConstraintSolver;
class btSimulationIslandManager;
class btSphereShape;
class btTypedConstraint;
class btActionInterface;

class btIDebugDraw;
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btDefaultMotionState.h"



#define NORMVEC_COMP		btScalar(0.577350269189625764)



///btDiscreteDynamicsWorld provides discrete rigid body simulation
///those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
class shGranularDiscreteDynamicsWorld : public btDiscreteDynamicsWorld
{
protected:

	btAlignedObjectArray<btSphereShape*> sphereShapes;

public:

	shGranularDiscreteDynamicsWorld( btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration );
	~shGranularDiscreteDynamicsWorld();

	// For setting up granular objects, which is, for each sphere, 8 additional smaller spheres are constrained to the center of that sphere
	virtual void addGranularSphere( btSphereShape* centerSphere, btVector3& centerSpherePosition, btScalar childSphereRadius );

};

#endif //BT_DISCRETE_DYNAMICS_WORLD_H
