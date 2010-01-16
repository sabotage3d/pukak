#ifndef SH_RIGIDBODY_H
#define SH_RIGIDBODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "btRigidBody.h"



class shRigidBody  : public btRigidBody
{
protected:
	//btTransform m_savedWorldTransform;	// I set this up in btCollisionObject.h

public:
	
	// CONSTRUCTORS
	shRigidBody(	const btRigidBodyConstructionInfo& constructionInfo );		// shRigidBody constructor using construction info
	shRigidBody(	btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia=btVector3(0,0,0));

	// public methods
	virtual void integrateVelocities( btScalar step );
	virtual void saveKinematicState( btScalar step );
	virtual void revertKinematicState();

};  // class shRigidBody



#endif
