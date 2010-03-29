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

#include "shCompoundSphereShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"




// CONSTRUCTOR (also the default constructor, since its only parameter has a default value set)
shCompoundSphereShape::shCompoundSphereShape( bool enableDynamicAabbTree )
: btCompoundShape( enableDynamicAabbTree )
{
	
}  // constructor



// CONSTRUCTOR
shCompoundSphereShape::shCompoundSphereShape( btAlignedObjectArray<btSphereShape*> sphereShapes, btAlignedObjectArray<btVector3> sphereRelativePositions, bool enableDynamicAabbTree )
: btCompoundShape( enableDynamicAabbTree )
{
	int numSpheres = sphereShapes.size();
	for ( int i = 0; i < numSpheres; i++ )
	{
		btSphereShape* curSphereShape = sphereShapes[i];
		btVector3 curRelPos = sphereRelativePositions[i];

		// Set up the relative transform based off the relative position.
		//   The transform's rotation will always be zero, since these are spheres and thus it would have no effect.
		btTransform curTransform;
		curTransform.setIdentity();
		curTransform.setOrigin( curRelPos );

		this->addChildShape( curTransform, curSphereShape );

	}  // for
}  // constructor

