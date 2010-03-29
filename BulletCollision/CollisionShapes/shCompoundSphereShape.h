
#ifndef COMPOUND_SPHERE_SHAPE_H
#define COMPOUND_SPHERE_SHAPE_H

#include "btCompoundShape.h"
#include "btSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"

// The btCompoundSphereShape implements a btCompoundShape made up of spheres.
ATTRIBUTE_ALIGNED16(class) shCompoundSphereShape : public btCompoundShape
{
	
public:
	
	// constructors
	shCompoundSphereShape( bool enableDynamicAabbTree = false );
	shCompoundSphereShape( btAlignedObjectArray<btSphereShape*> sphereShapes, btAlignedObjectArray<btVector3> sphereRelativePositions, bool enableDynamicAabbTree = false );
	
};  // btCompoundSphereShape


#endif // COMPOUND_SPHERE_SHAPE_H
