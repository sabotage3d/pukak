#ifndef SH_SOLVER_BODY_H
#define SH_SOLVER_BODY_H


#include "btSolverBody.h"


///The btSolverBody is an internal datastructure for the constraint solver. Only necessary data is packed to increase cache coherence/performance.
ATTRIBUTE_ALIGNED16 (struct) shSolverBody : btSolverBody
{
//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	SIMD_FORCE_INLINE void applyImpulse( const btVector3& linearComponent, const btVector3& angularComponent, const btScalar impulseMagnitude = 0.f )
	{
		//if (m_invMass)
		{
			m_deltaLinearVelocity += linearComponent;
			m_deltaAngularVelocity += angularComponent;
		}
	}
};  // struct shSolverBody

#endif
