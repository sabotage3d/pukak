#include "shRigidBody.h"

#include "LinearMath/btTransformUtil.h"

#include <stdio.h>




// CONSTRUCTORS //

shRigidBody::shRigidBody( const btRigidBody::btRigidBodyConstructionInfo& constructionInfo )
:btRigidBody( constructionInfo )
{
}  // CONSTRUCTOR




shRigidBody::shRigidBody( btScalar mass, btMotionState *motionState, btCollisionShape *collisionShape, const btVector3 &localInertia )
:btRigidBody( mass, motionState, collisionShape, localInertia )
{
}  // CONSTRUCTOR




// OTHER METHODS

void shRigidBody::integrateVelocities( btScalar step )
{
	if (isStaticOrKinematicObject())
		return;
	
	// update linear and angular velocities useing Euler's method
	m_linearVelocity += m_totalForce * ( m_inverseMass * step );
	m_angularVelocity += m_invInertiaTensorWorld * m_totalTorque * step;
	
	#define MAX_ANGVEL SIMD_HALF_PI

	// clamp angular velocity. collision calculations will fail on higher angular velocities	
	btScalar angvel = m_angularVelocity.length();
	if (angvel*step > MAX_ANGVEL) {

		m_angularVelocity *= ( MAX_ANGVEL / step ) / angvel;

	}  // if

}  // integrateVelocities( btScalar )




void shRigidBody::saveKinematicState( btScalar timeStep )
{
	// save out the transform and velocity exactly as they are
	m_savedWorldTransform = m_worldTransform;
	
	btRigidBody::saveKinematicState( timeStep );	// this updates the m_interpolationXX variables

}  // saveKinematicState()




void shRigidBody::revertKinematicState()
{
	// save out the transform and velocity exactly as they are
	m_worldTransform = m_savedWorldTransform;
	
}  // saveKinematicState()


