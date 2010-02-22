
#ifndef SPHERE_GRANULE_H
#define SPHERE_GRANULE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btSphereShape implements an implicit sphere, centered around a local origin with radius.
ATTRIBUTE_ALIGNED16(class) btGranuleSphereShape : public btSphereShape

{
	
public:
	
	btGranuleSphereShape( btScalar radius ) : btSphereShape(radius)
	{
	}
	
	shGranule myGranule;

};


#endif //SPHERE_MINKOWSKI_H
