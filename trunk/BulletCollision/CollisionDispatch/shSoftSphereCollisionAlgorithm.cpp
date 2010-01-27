#include "shSoftSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"


// CONSTRUCTOR //
shSoftSphereCollisionAlgorithm::shSoftSphereCollisionAlgorithm( btPersistentManifold* mf, const btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* col0, btCollisionObject* col1 )
: btActivatingCollisionAlgorithm( ci, col0, col1 ),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	if ( !m_manifoldPtr )
	{
		m_manifoldPtr = m_dispatcher->getNewManifold( col0, col1 );
		m_ownManifold = true;
	}

}  // constructor




// DESTRUCTOR //
shSoftSphereCollisionAlgorithm::~shSoftSphereCollisionAlgorithm()
{
	if ( m_ownManifold )
	{
		if ( m_manifoldPtr )
			m_dispatcher->releaseManifold( m_manifoldPtr );
	}

}  // destructor




void shSoftSphereCollisionAlgorithm::processCollision( btCollisionObject* col1, btCollisionObject* col2, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut )
{
	(void)dispatchInfo;

	if ( !m_manifoldPtr )
		return;

	resultOut->setPersistentManifold(m_manifoldPtr);

	btSphereShape* sphere1 = (btSphereShape*)col1->getCollisionShape();
	btSphereShape* sphere2 = (btSphereShape*)col2->getCollisionShape();

	btVector3 diff = col1->getWorldTransform().getOrigin() - col2->getWorldTransform().getOrigin();
	btScalar len = diff.length();
	btScalar radius1 = sphere1->getRadius();
	btScalar radius2 = sphere2->getRadius();

	#ifdef CLEAR_MANIFOLD
	m_manifoldPtr->clearManifold(); //don't do this, it disables warmstarting
	#endif
	///iff distance positive, don't generate a new contact
	if ( len > (radius1+radius2))
	{
	#ifndef CLEAR_MANIFOLD
		resultOut->refreshContactPoints();
	#endif //CLEAR_MANIFOLD
		return;
	}
	
	// set the overlap of the spheres (greek letter Xi)
	btScalar Xi = radius1 + radius2 - len;

	// set the normalized line of centers from sphere2 (B) center to sphere2 (A) center
	btVector3 normalOnSurface2(1,0,0);
	if (len > SIMD_EPSILON)
	{
		normalOnSurface2 = diff / len;
	}

	btVector3 collisionPosition = col2->getWorldTransform().getOrigin() + radius2 * normalOnSurface2;

	resultOut->addContactPoint( normalOnSurface2, collisionPosition, Xi );	// Creates a btManifoldPoint and adds it to m_pointCache array in m_manifoldPtr (class btPersistentManifold) of resultOut (btManifoldResult class).

	#ifndef CLEAR_MANIFOLD
	resultOut->refreshContactPoints();
	#endif //CLEAR_MANIFOLD

}  // processCollision()





