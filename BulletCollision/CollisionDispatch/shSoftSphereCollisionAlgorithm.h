#ifndef SOFT_SPHERE_COLLISION_ALGORITHM_H
#define SOFT_SPHERE_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "btCollisionDispatcher.h"

class btPersistentManifold;


/// shSoftSphereCollisionAlgorithm provides a Molecular-Dynamics implementation
//   of sphere-sphere collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
class shSoftSphereCollisionAlgorithm : public btActivatingCollisionAlgorithm
{
	bool	m_ownManifold;
	btPersistentManifold*	m_manifoldPtr;

public:
	shSoftSphereCollisionAlgorithm( btPersistentManifold* mf, const btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0, btCollisionObject* body1 );
	shSoftSphereCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci )
		: btActivatingCollisionAlgorithm( ci ) {}

	virtual ~shSoftSphereCollisionAlgorithm();
	
	virtual void processCollision ( btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut );
	virtual btScalar calculateTimeOfImpact( btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut );


	virtual	void getAllContactManifolds( btManifoldArray& manifoldArray )
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}  // getAllContactManifolds


	struct CreateFunc : public btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(shSoftSphereCollisionAlgorithm));
			return new(mem) shSoftSphereCollisionAlgorithm(0,ci,body0,body1);
		}
	};

};  // class shSoftSphereCollisionAlgorithm

#endif
