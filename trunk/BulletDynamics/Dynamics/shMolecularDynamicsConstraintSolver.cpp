#include "shMolecularDynamicsConstraintSolver.h"

#include "LinearMath/btQuickprof.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include <math.h>

#include <iostream>
using std::cout;
using std::endl;




shMolecularDynamicsConstraintSolver::shMolecularDynamicsConstraintSolver()
{
	CollisionHappened = 0.f;
}

shMolecularDynamicsConstraintSolver::~shMolecularDynamicsConstraintSolver()
{
}

// Required by abstract parent class definition
void shMolecularDynamicsConstraintSolver::reset()
{
	CollisionHappened = 0.f;

}  // reset()




btScalar shMolecularDynamicsConstraintSolver::solveGroup( btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc, btDispatcher* /*dispatcher*/)
{
	BT_PROFILE("solveGroup");
	//we only implement SOLVER_CACHE_FRIENDLY now
	//you need to provide at least some bodies
	btAssert(bodies);
	btAssert(numBodies);

	// Iterate through all manifolds in manifoldPtr, computing each manifold's collision impulse for the deepest non-diverging point
	//   For each manifold, the computed impulse is used to update the velocities of BOTH bodies in the manifold
	btScalar doRepeatCollisionsStep = solveCollisionConstraints( manifoldPtr,  numManifolds, infoGlobal, debugDrawer, stackAlloc );
	
	// RESET all of the btConstraintSolver and btSolverBody arrays
	m_tmpSolverBodyPool.resize(0);
	m_tmpSolverContactConstraintPool.resize(0);
	m_tmpSolverNonContactConstraintPool.resize(0);
	//m_tmpSolverContactFrictionConstraintPool.resize(0);

	return doRepeatCollisionsStep;	// If doRepeatCollisionsStep == 1.f, then the dynamics world knows to reset the objects' positions based on the new velocities and run collsions again

}  // solveGroup()



// solveCollisionConstraints()
//   This is the core loop of the nonconvex stacking collision algorithm, to go through each manifold and update the velocities of the respective colliding pairs
btScalar shMolecularDynamicsConstraintSolver::solveCollisionConstraints( btPersistentManifold** manifoldPtr, int numManifolds, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc )
{
	BT_PROFILE("solveCollisionConstraints");
	(void)stackAlloc;
	(void)debugDrawer;

	shSolverBody& fixedBody = m_tmpSolverBodyPool.expand();			// m_tmpSolverBodyPool is type btAlignedObjectArray<btSolverBody>
	initSolverBody( &fixedBody, 0 );		// in this case, 0 passed in for btCollisionObject*, so mass,etc. are set to zero instead of an object's mass

	if ( !numManifolds )
	{
		return 0.f;

	}  // if
//printf( "Num manifolds = %d\n", numManifolds );
	CollisionHappened = 0.f;

	// For each collision manifold, 
	// There is a btPersistentManifold object (manifold) for each collision in the current step
	//    There is a manifold for each pair of colliding objects
	int numContacts = 0;
	for ( int i = 0; i < numManifolds; i++ )
	{
		btPersistentManifold* manifold = manifoldPtr[i];
		btScalar foundAndComputedCollision = 0.f;

		btVector3 rBTot( 0, 0, 0 );
		int numContacts = manifold->getNumContacts();
		btCollisionObject* colObjB = (btCollisionObject*)manifold->getBody1();	// btPersistentManifold has getBody0() and getBody1(), so this grabs object B
		
		/*for ( int j = 0; j < numContacts; j++ )
		{
			btManifoldPoint& cp = manifold->getContactPoint( j );
			const btVector3& posB = cp.getPositionWorldOnB();
			btVector3 rB = posB - colObjB->getWorldTransform().getOrigin();
			printf( "rB %d = %f %f %f\n", j, rB.x(), rB.y(), rB.z() );
			rBTot = btVector3( rBTot.x() + rB.x(), rBTot.y() + rB.y(), rBTot.z() + rB.z() );

		}  // for j*/
		
		// compute impulse and resulting deltas for linear and angular velocities
				
		// Compute impulses and UPDATE VELOCITIES for colliding objects based on each collision point in the manifold.
		//   Each collision point that is diverging is removed from the manifold
		computeSoftSphereCollision( manifold, infoGlobal );
		
	}  // for i

	return CollisionHappened;		// If a collision happened, this process will need to be repeated.

}  // solveCollisionConstraints()




// computeContactForDeepestNondivergingPointOnManifold()
//   Returns 0.0 if there is NO collision
//   Returns 1.0 if there IS a collision
btScalar shMolecularDynamicsConstraintSolver::computeSoftSphereCollision( btPersistentManifold* manifold, const btContactSolverInfo& infoGlobal )
{
	// Extract the colliding objects from the manifold
	btCollisionObject* colObjA = (btCollisionObject*)manifold->getBody0();
	btCollisionObject* colObjB = (btCollisionObject*)manifold->getBody1();
	btRigidBody* rbA = btRigidBody::upcast(colObjA);
	btRigidBody* rbB = btRigidBody::upcast(colObjB);
	
	
	// Set up a solver body (or retrieve an already existing one) for both colliding bodies.
	//   The solver bodies are stored in m_tmpSolverBodyPool
	int solverBodyIdA = -1;
	int solverBodyIdB = -1;
	int numContacts = manifold->getNumContacts();	// All we care about is a single contact point
	if ( numContacts )
	{
		solverBodyIdA = getOrInitSolverBody( *colObjA );	// Get the ID for object A's solverBody.  If no solverBody exists for object A, create a new one, assign its ID to the object, and return the new ID
		solverBodyIdB = getOrInitSolverBody( *colObjB );	// Get the ID for object B's solverBody.  If no solverBody exists for object B, create a new one, assign its ID to the object, and return the new ID

	}  // if

	/// avoid collision response between two static objects
	if (!solverBodyIdA && !solverBodyIdB)
		return btScalar(0.0);		// 0.0 means no collision, we may eventually need another return option, such as -1.0 to report a static pair

	// Set up the solverConstraint, for passing data around into the various methods called in this method
	//   A btSolverConstraint keeps track of values that define the constraint (eg. friction, impulse)
	//   In this implementation, we only need one btSolverConstraint per manifold, since we use only one collision pair in the manifold as the collision constraint
	btSolverConstraint& solverConstraint = m_tmpSolverContactConstraintPool.expand();
	solverConstraint.m_solverBodyIdA = solverBodyIdA;
	solverConstraint.m_solverBodyIdB = solverBodyIdB;
	// printf( "num contacts = %d\n", numContacts );


	

	// Grab the deepest collision point (first in the point array) in the manifold
	//   Since we're dealing with spheres, this is all that matters
	btManifoldPoint& cp = manifold->getContactPoint( 0 );
	btVector3 contactNormal = -1 * cp.m_normalWorldOnB;	// The collision normal is inverted here because we want to go from A to B (A will have the relative velocity).
	contactNormal = contactNormal.normalize();
	btScalar invMassA = rbA->getInvMass();
	btScalar invMassB = rbB->getInvMass();

	// Compute relative velocity of colliding objects, and split it into normal and tangential components
	btScalar XiContactDepth = cp.getDistance();
	btVector3 linVel1 = rbA->getLinearVelocity();
	btVector3 linVel2 = rbB->getLinearVelocity();
	btVector3 relVel = linVel1 - linVel2;				// Relative velocity vector, of sphere1 (as if sphere2 were at rest)
	btScalar relVelNMag = relVel.dot( contactNormal );	// Velocity vector magnitude in the normal (Xi) direction
	btVector3 relVelN = relVelNMag * contactNormal;
	btVector3 relVelT = relVel - relVelN;
	btScalar relVelTMag = relVelT.length();

	// NORMAL FORCES COMPUTATION

	// Set up variables for the normal forces of the collision, which are treated as spring forces
	btScalar alpha = 0.0;	// determines whether relative position is included into the velocity term of the spring equation
	btScalar beta = 1.0;	// to control linearity of the force function
	
	// Spring constants
	btScalar e_n = 1.0;	// Coefficient of normal restitution
	btScalar t_c = 1.0;	// Contact duration
	btScalar log_e_n = log(e_n);
	btScalar m_eff = btScalar(1.0) / ( invMassA + invMassB );	// Reduced mass

	// Compute damping coefficient, kd, which influences the velocity term of the spring force equation
	btScalar k_d = 2 * m_eff * ( -log_e_n / t_c );

	// Compute elastic restoration coefficient, kr, which influences the penetration depth (also known as the "spring constant")
	btScalar k_r = ( m_eff / t_c*t_c ) * ( log_e_n*log_e_n + PI_SQUARED );

	// Compute relative velocity term of the spring equation
	btScalar relativeVelocityTerm = k_d * relVelNMag;
	if ( alpha != 0.0 ) relativeVelocityTerm = relativeVelocityTerm * pow( XiContactDepth, alpha );

	// Compute relative position term of the spring equation
	if ( beta != 1.0 ) XiContactDepth = pow( XiContactDepth, beta );
	btScalar relativePositionTerm = k_r * XiContactDepth;

	// Compute the normal force (with the spring force equation)
	btScalar F_n = -1 * ( relativeVelocityTerm + relativePositionTerm );	// NORMAL FORCE

	// TANGENTIAL (SHEAR) FORCES COMPUTATION

	// Set up variables for the tangential forces of the collision (created by friction)
	btScalar mu = 1.0;	// Friction coefficient
	
	btScalar k_t = 1.0;	// Viscous damping constant

	btScalar dampingTerm = -1 * min( (mu * F_n), (k_t*relVelTMag) );

	// Compute the tangential force
	btScalar F_t = dampingTerm * (relVelT/relVelTMag);





	// Set up the contact constraints
	//   This loops through each pair of contact points in the manifold
	//   If this loop finds a converging (colliding) pair of points, it computes the impulse for that pair then returns out of this method (loop over), reporting a collision (returns 1.0)
	//   Else, all pairs of points are diverging (no collision) and this method returns 0.0 (no converging points)
	for ( int j = 0; j < numContacts; j++ )
	{
		// "manifold," of type btPersistentManifold, is a contact point cache (contains btManifoldPoint objects),
		//    and it stays persistent as long as objects are overlapping in the broadphase.
		// A btManifoldPoint contains (and maintains) the positions of the two points in the contact (one for each object)
		//    as well as correlated values such as friction and restitution
		btManifoldPoint& cp = manifold->getContactPoint( j );
							
		// find or create solverBodys matching the rigidBodys
		//int solverBodyIdA = getOrInitSolverBody(rbA);		// a btSolverBody is a struct representing the body (object) and solves/computes change in velocities based off impulse,etc.; points back to its corresponding btRigidBody
		//int solverBodyIdB = getOrInitSolverBody(rbB);

		



		
		// Local velocity computation
		//   Computes local collision point positions relative to the centers of objects A and B respectively
		const btVector3& posA = cp.getPositionWorldOnA();
		const btVector3& posB = cp.getPositionWorldOnB();
		btVector3 rA = posA - colObjA->getWorldTransform().getOrigin();		// rA = the vector from the center of mass (COM) to the collision point on object A
		btVector3 rB = posB - colObjB->getWorldTransform().getOrigin();		// rB = the vector from the center of mass (COM) to the collision point on object B
		
		//rB = btVector3( 0.0, rBTot.y(), 0.0 );

		solverConstraint.m_relPos1 = rA;
		solverConstraint.m_relPos2 = rB;
		solverConstraint.m_friction = cp.m_combinedFriction;
		solverConstraint.m_contactNormal = cp.m_normalWorldOnB;

		// Calculate local velocities of the collision points and their relative velocity (the difference between the two points' velocities)
		computeVelocitiesForCollisionPair( rbA, rbB, solverConstraint );
		
		// COMPUTE WHETHER THE COLLIDING POINTS ARE HEADING TOWARD EACH OTHER (CONVERGING) OR NOT (DIVERGING)
		// If the relative velocity is headed opposite the collision normal (the normal at point B)
		//   then the dot product is negative, meaning the two points are colliding (converging);
		//   otherwise if the dot product is positive, the two points are diverging and thus there is no collision
		if ( solverConstraint.m_relVelNormalMagnitude >= 0.0 )		// diverging (if the points are diverging, there is no collision)
		{
			manifold->removeContactPoint( j );
			continue;

		}  // if
		else													// converging (the points are colliding, or converging toward the separating plane)
		{
			// Compute the impulse vector for the current contact point
			computeImpulseForCollisionPair( rbA, rbB, solverConstraint );		// Assigns the final impulse to solverConstraint.m_impulse
			
			// Compute the delta velocities of the objects based on solverConstraint.m_impulse
			applyImpulseForCollisionPair( rbA, rbB, solverConstraint );

			updateVelocitiesForCollisionPair( solverBodyIdA, solverBodyIdB, rbA, rbB, infoGlobal.m_timeStep );
			
			manifold->removeContactPoint( j );
			return btScalar(1.f);	// Returning 1.0 flags that we resolved a collision and thus will have to iterate through all the manifolds again

		}  // else

	}  // for j

	// If you got here in the code, all points in the manifold were diverging
	return btScalar(0.f);

}  // computeContactForDeepestNondivergingPointOnManifold()




// computeVelocitiesForCollisionPair()
//   The solverConstraint MUST have the following attributes already set:
//     m_relPos1
//     m_relPos2
//     m_contactNormal
//   This method will set the following solverConstraint attributes:
//     m_relativeVelocity
//     m_relVelNormalMagnitude
void shMolecularDynamicsConstraintSolver::computeVelocitiesForCollisionPair( btRigidBody* rbA, btRigidBody* rbB, btSolverConstraint& solverConstraint )
{
	btVector3 rA = solverConstraint.m_relPos1;
	btVector3 rB = solverConstraint.m_relPos2;
	//btVector3 linvel1 = rbA->getLinearVelocity();
	btVector3 linvel2 = rbB->getLinearVelocity();
	//btVector3 linvel1 = rbA->getLinearVelocity();
	btVector3 angvel2 = rbB->getAngularVelocity();
	//printf( "m_linearVelocity A = %f %f %f\n", linvel1[0], linvel1[1], linvel1[2] );
	printf( "1.m_linearVelocity B = %f %f %f\n", linvel2[0], linvel2[1], linvel2[2] );
	printf( "1.m_angularVelocity B = %f %f %f\n", angvel2[0], angvel2[1], angvel2[2] );
	btVector3 velA = rbA ? rbA->getVelocityInLocalPoint( rA ) : btVector3( 0,0,0 );		// Get the velocity at the given position on surface A (adds the linear and angular velocities at that point)
	btVector3 velB = rbB ? rbB->getVelocityInLocalPoint( rB ) : btVector3( 0,0,0 );		// Get the velocity at the given position on surface B (adds the linear and angular velocities at that point)
	//printf( "rA = %f %f %f\n", rA.x(), rA.y(), rA.z() );
	//printf( "rB = %f %f %f\n", rB.x(), rB.y(), rB.z() );
	//printf( "VelA = %f %f %f\n", velA.x(), velA.y(), velA.z() );
	//printf( "VelB = %f %f %f\n", velB.x(), velB.y(), velB.z() );
	// COMPUTE RELATIVE VELOCITY OF THE COLLISION POINT
	// Compute the relative velocity of the two collision points
	// The variable uRelNormalMagnitude will represent both the magnitude of the point's velocity in the normal direction
	//   as well as the relative direction of the colliding points (whether they are coming together or moving apart, converging or diverging).
	btVector3 uRel = velA - velB;	// uRel is the relative velocity
	btScalar uRelN = solverConstraint.m_contactNormal.dot( uRel );		// uRel is the relative velocity, and uRelN is uRel's magnitude (scalar) in the normal direction
	//printf( "Relative velocity = %f %f %f\n", uRel.x(), uRel.y(), uRel.z() );
	// Set the solverConstraint values for passing into computeImpulseForCollisionPair()
	solverConstraint.m_relativeVelocity = uRel;
	solverConstraint.m_relVelNormalMagnitude = uRelN;

}  // computeVelocitiesForCollisionPair()




// computeImpulseForCollisionPair()
//   The solverConstraint MUST have the following attributes already set:
//      m_relPos1
//      m_relPos2
//      m_contactNormal
//      m_relativeVelocity
//      m_relVelNormalMagnitude (currently, since uRelN had to be computed in the parent method, this method does not recompute it even though it could do so)
//      m_friction
//   Assigns the final impulse to solverConstraint.m_impulse
void shMolecularDynamicsConstraintSolver::computeImpulseForCollisionPair( btRigidBody* rbA, btRigidBody* rbB, btSolverConstraint& solverConstraint )
{
	btVector3 rA = solverConstraint.m_relPos1;
	btVector3 rB = solverConstraint.m_relPos2;
	btVector3 collisionNormal = solverConstraint.m_contactNormal;
	//printf( "Contact Normal = %f %f %f\n", collisionNormal.x(), collisionNormal.y(), collisionNormal.z() );
	btVector3 uRel = solverConstraint.m_relativeVelocity;
	btScalar uRelN = solverConstraint.m_relVelNormalMagnitude;
	btScalar friction = solverConstraint.m_friction;

	// the restitution (epsilon) equals the lesser restitution of the two objects
	btScalar restitution = 0.f;
	btScalar restitutionA = rbA->getRestitution();
	btScalar restitutionB = rbB->getRestitution();
	if ( restitutionA < restitutionB )
		restitution = restitutionA;
	else
		restitution = restitutionB;

	// K-matrix calculation.
	//   We already have rA and rB, now we need to compute rA* and rB*, which are the matrix representations for doing the cross product
	btMatrix3x3 rCrossMatrixA(     0.0,  -rA[2],   rA[1],
	                             rA[2],     0.0,  -rA[0],
						        -rA[1],   rA[0],      0.0  );

	btMatrix3x3 rCrossMatrixB(     0.0,  -rB[2],   rB[1],
	                             rB[2],     0.0,  -rB[0],
						        -rB[1],   rB[0],      0.0  );
	
	// Linear velocity term (delta/mass), where delta = the identity matrix, and mass = the mass of each object.
	//   This inverse mass matrix multiplied by impulse = the deltaLinearVelocity
	float iMassA = rbA->getInvMass();
	float iMassB = rbB->getInvMass();
	btMatrix3x3 invMassMatrixA( iMassA,     0.0,     0.0,
							       0.0,  iMassA,     0.0,
							       0.0,     0.0,   iMassA  );

	btMatrix3x3 invMassMatrixB( iMassB,     0.0,     0.0,
							       0.0,  iMassB,     0.0,
							       0.0,     0.0,   iMassB  );
	//printf( "Mass B, Inv Mass B = %f %f\n", 1.0/iMassB, iMassB );

	// Angular velocity term ((r*)(Iinv)(r*)), where r* is the cross product matrix computed above, and Iinv = inverse of the inertia tensor (the angular momentum of the object).
	btMatrix3x3 invMomentOfInertiaA = rbA->getInvInertiaTensorWorld();
	btMatrix3x3 invMomentOfInertiaB = rbB->getInvInertiaTensorWorld();

	//printf("Moment of Inertia:\n");
	//printf("  %f %f %f\n", invMomentOfInertiaB.getRow(0).x(), invMomentOfInertiaB.getRow(0).y(), invMomentOfInertiaB.getRow(0).z() );
	//printf("  %f %f %f\n", invMomentOfInertiaB.getRow(1).x(), invMomentOfInertiaB.getRow(1).y(), invMomentOfInertiaB.getRow(1).z() );
	//printf("  %f %f %f\n", invMomentOfInertiaB.getRow(2).x(), invMomentOfInertiaB.getRow(2).y(), invMomentOfInertiaB.getRow(2).z() );
	
	btMatrix3x3 KA = invMassMatrixA + ( rCrossMatrixA.transpose() * invMomentOfInertiaA * rCrossMatrixA );
	btMatrix3x3 KB = invMassMatrixB + ( rCrossMatrixB.transpose() * invMomentOfInertiaB * rCrossMatrixB );
	btMatrix3x3 K = KA + KB;
	
	//printf("KA:\n");
	//printf("  %f %f %f\n", KA.getRow(0).x(), KA.getRow(0).y(), KA.getRow(0).z() );
	//printf("  %f %f %f\n", KA.getRow(1).x(), KA.getRow(1).y(), KA.getRow(1).z() );
	//printf("  %f %f %f\n", KA.getRow(2).x(), KA.getRow(2).y(), KA.getRow(2).z() );
	//printf("KB:\n");
	//printf("  %f %f %f\n", KB.getRow(0).x(), KB.getRow(0).y(), KB.getRow(0).z() );
	//printf("  %f %f %f\n", KB.getRow(1).x(), KB.getRow(1).y(), KB.getRow(1).z() );
	//printf("  %f %f %f\n", KB.getRow(2).x(), KB.getRow(2).y(), KB.getRow(2).z() );
	
	// Friction calculation.
	//   We are using BULLET's method (for friction only).
	btVector3 uRelNormal = collisionNormal * uRelN;		// normal-component of the collision point's relative velocity, uRel
	btVector3 uRelTangent = uRel - uRelNormal;			// tangent-component of the collision point's relative velocity, uRel

	// Determine if friction should be included in the impulse computation.
	//   **** NOT COMPLETED ****
	bool applyFriction = false;		// To be determined in more detail later, for now we are ignoring friction, though some of the math is already implemented in NAdjustedForFriction above

	// Impulse calculation.
	//btScalar finalURelN = btScalar(-1.0) * restitution * uRelN;
	btVector3 NK = collisionNormal * K;			// Properly does "row matrix * matrix" => row matrix
	btScalar NKN;
	if ( applyFriction )
	{
		btVector3 T = uRelTangent / uRelTangent.length();
		btVector3 NAdjustedForFriction = collisionNormal - T * friction;
		NKN = NK.dot( NAdjustedForFriction );	// Properly does "row matrix * column matrix" (in other words, vector.vector)
	}  // if
	else
		NKN = NK.dot( collisionNormal );	// Properly does "row matrix * column matrix" (in other words, vector.vector)
	
	//printf( "uRelN = %f\n", uRelN );
	printf( "restitution = %f\n", restitution );
	//printf( "NKN = %f\n", NKN );

	btScalar jN = ( btScalar(-1.0) * uRelN * (restitution + btScalar(1.0)) )  /  NKN;	// jN is the IMPULSE MAGNITUDE

	solverConstraint.m_impulse = jN * collisionNormal;	// FINAL IMPULSE CALCULATION

	//printf( "  IMPULSE = %f %f %f\n", solverConstraint.m_impulse[0], solverConstraint.m_impulse[1], solverConstraint.m_impulse[2] );
	
}  // computeImpulseForCollisionPair()




void shMolecularDynamicsConstraintSolver::applyImpulseForCollisionPair( btRigidBody* rbA, btRigidBody* rbB, btSolverConstraint& solverConstraint )
{//printf( "applying impulses\n" );
	btScalar iMassA = rbA->getInvMass();
	btScalar iMassB = rbB->getInvMass();
	
	// Compute the deltas in both linear and angular velocities for both objects in the collision pair
	btVector3 deltaLinearVelocityA = solverConstraint.m_impulse * iMassA;	// multiply the impulse times 1/massA (the mass's inverse)
	btVector3 deltaLinearVelocityB = btScalar(-1.f)*solverConstraint.m_impulse * iMassB;	// multiply the impulse times 1/massB (the mass's inverse)

	btVector3 rA = solverConstraint.m_relPos1;
	btVector3 rB = solverConstraint.m_relPos2;
	btVector3 deltaAngularVelocityA = rbA->getInvInertiaTensorWorld() * ( rA.cross(solverConstraint.m_impulse) );	// matrix * column matrix (rA.cross(impulse) returns a vector that is properly treated as a column matrix)
	btVector3 deltaAngularVelocityB = rbB->getInvInertiaTensorWorld() * ( rB.cross(btScalar(-1.f)*solverConstraint.m_impulse) );	// matrix * column matrix (rB.cross(impulse) returns a vector that is properly treated as a column matrix)
	//printf( "The delta angular velocity of B = %f %f %f\n", deltaAngularVelocityB[0], deltaAngularVelocityB[1], deltaAngularVelocityB[2] );
	// Update the deltas for the linear and angular velocities
	// btVector3(0,0,0) );//
	m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdA].applyImpulse( deltaLinearVelocityA, deltaAngularVelocityA );	// In updateVelocities(), solverBody.writebackVelocity() will be called to apply these updates
	m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdB].applyImpulse( deltaLinearVelocityB, deltaAngularVelocityB );	// In updateVelocities(), solverBody.writebackVelocity() will be called to apply these updates

}  // applyImpulseForCollisionPair()




void shMolecularDynamicsConstraintSolver::updateVelocitiesForCollisionPair( int objectIdA, int objectIdB, btRigidBody* rbA, btRigidBody* rbB, btScalar timeStep ) {

	// writebackVelocity() adds m_deltaLinearVelocity to this object's linear velocity and m_deltaAngularVelocity to this objects angular velocity
	//   and updates the corresponding rigid body's linear and angular velocities
	m_tmpSolverBodyPool[objectIdA].writebackVelocity();
	m_tmpSolverBodyPool[objectIdB].writebackVelocity();

	btVector3 linvel2 = rbB->getLinearVelocity();
	btVector3 angvel2 = rbB->getAngularVelocity();
	printf( "2.m_linearVelocity B = %f %f %f\n", linvel2[0], linvel2[1], linvel2[2] );
	printf( "2.m_angularVelocity B = %f %f %f\n", angvel2[0], angvel2[1], angvel2[2] );

	/*if ( !rbA->isStaticOrKinematicObject() )
	{
		btTransform predictedTrans;
		rbA->predictIntegratedTransform( timeStep, predictedTrans );
		rbA->updateInertiaTensor( predictedTrans );

	}  // if
	if ( !rbB->isStaticOrKinematicObject() )
	{
		btTransform predictedTrans;
		printf("\n");
		printf("Before:\n");
		printf("  %f %f %f\n", predictedTrans.getBasis().getRow(0).x(), predictedTrans.getBasis().getRow(0).y(), predictedTrans.getBasis().getRow(0).z() );
		printf("  %f %f %f\n", predictedTrans.getBasis().getRow(1).x(), predictedTrans.getBasis().getRow(1).y(), predictedTrans.getBasis().getRow(1).z() );
		printf("  %f %f %f\n", predictedTrans.getBasis().getRow(2).x(), predictedTrans.getBasis().getRow(2).y(), predictedTrans.getBasis().getRow(2).z() );
		printf("\n");
		rbB->predictIntegratedTransform( timeStep, predictedTrans );
		rbB->updateInertiaTensor( predictedTrans );
		printf("After:\n");
		printf("  %f %f %f\n", predictedTrans.getBasis().getRow(0).x(), predictedTrans.getBasis().getRow(0).y(), predictedTrans.getBasis().getRow(0).z() );
		printf("  %f %f %f\n", predictedTrans.getBasis().getRow(1).x(), predictedTrans.getBasis().getRow(1).y(), predictedTrans.getBasis().getRow(1).z() );
		printf("  %f %f %f\n", predictedTrans.getBasis().getRow(2).x(), predictedTrans.getBasis().getRow(2).y(), predictedTrans.getBasis().getRow(2).z() );
		printf("\n");

	}  // if*/

}  // updateVelocities()




// solveBodyConstraints()
//   Copied over from the body constraints portion of btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup()
btScalar shMolecularDynamicsConstraintSolver::solveNonContactConstraints( btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc )
{
	if ( !numConstraints )		// if no constraints and no collisions
	{
		return 0.f;

	}  // if
	
	// Constraints are manually added when the scene is built,
	//    so for scenes like BasicDemo where no constraints are defined, numConstraints = 0, and this for-loop is not entered
	int j;
	for ( j=0; j<numConstraints; j++ )
	{
		btTypedConstraint* constraint = constraints[j];
		constraint->buildJacobian();
	}
	
	// expand() adds another btSolverBody to the end of m_tmpSolverBodyPool and returns that end btSolverBody, then initSolverBody() inits that btSolverBody's values
	shSolverBody& fixedBody = m_tmpSolverBodyPool.expand();			// m_tmpSolverBodyPool is type btAlignedObjectArray<btSolverBody>
	initSolverBody( &fixedBody, 0 );		// in this case, 0 passed in for btCollisionObject*, so mass,etc. are set to zero instead of an object's mass

	//btRigidBody* rb0=0,*rb1=0;
	
	// set up and solve the constraints (see btTypedConstraint.h for the enumeration of constraint types)
	//    Again, scenes won't have any constraints (such as BasicDemo) unless they are hard coded into the scene (such as in RagdollDemo)
	{

		int totalNumRows = 0;
		int i;
		//calculate the total number of contraint rows
		for ( i=0; i<numConstraints; i++ )
		{
			btTypedConstraint::btConstraintInfo1 info1;
			constraints[i]->getInfo1(&info1);
			totalNumRows += info1.m_numConstraintRows;
		}  // for i
		m_tmpSolverNonContactConstraintPool.resize(totalNumRows);

		btTypedConstraint::btConstraintInfo1 info1;
		info1.m_numConstraintRows = 0;

		// setup the btSolverConstraints
		//   
		int currentRow = 0;

		for ( i=0; i<numConstraints; i++,currentRow+=info1.m_numConstraintRows )
		{
			constraints[i]->getInfo1(&info1);
			if (info1.m_numConstraintRows)
			{
				btAssert(currentRow<totalNumRows);

				btSolverConstraint* currentConstraintRow = &m_tmpSolverNonContactConstraintPool[currentRow];
				btTypedConstraint* constraint = constraints[i];

				btRigidBody& rbA = constraint->getRigidBodyA();		// a btRigidBody is a class that keeps track of and updates the rigid body object's current attributes
				btRigidBody& rbB = constraint->getRigidBodyB();
					
				// find or create solverBodys matching the rigidBodys
				int solverBodyIdA = getOrInitSolverBody(rbA);		// a btSolverBody is a struct representing the body (object) and solves/computes change in velocities based off impulse,etc.; points back to its corresponding btRigidBody
				int solverBodyIdB = getOrInitSolverBody(rbB);

				btSolverBody* bodyAPtr = &m_tmpSolverBodyPool[solverBodyIdA];
				btSolverBody* bodyBPtr = &m_tmpSolverBodyPool[solverBodyIdB];
					
				// Each typeConstraint (constraint) has multiple corresponding solverConstraints (array currentConstraintRow)
				//    The number of solverConstraints needed by the current typeConstraint is determined by info1, which was set up by typeConstraint->getInfo1()
				int j;
				for ( j=0; j<info1.m_numConstraintRows; j++ )
				{
					memset(&currentConstraintRow[j],0,sizeof(btSolverConstraint));
					currentConstraintRow[j].m_lowerLimit = -FLT_MAX;
					currentConstraintRow[j].m_upperLimit = FLT_MAX;
					currentConstraintRow[j].m_appliedImpulse = 0.f;
					currentConstraintRow[j].m_appliedPushImpulse = 0.f;
					currentConstraintRow[j].m_solverBodyIdA = solverBodyIdA;
					currentConstraintRow[j].m_solverBodyIdB = solverBodyIdB;
				}

				bodyAPtr->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
				bodyAPtr->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);
				bodyBPtr->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
				bodyBPtr->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);

				btTypedConstraint::btConstraintInfo2 info2;
				info2.fps = 1.f/infoGlobal.m_timeStep;
				info2.erp = infoGlobal.m_erp;
				info2.m_J1linearAxis = currentConstraintRow->m_contactNormal;
				info2.m_J1angularAxis = currentConstraintRow->m_relpos1CrossNormal;
				info2.m_J2linearAxis = 0;
				info2.m_J2angularAxis = currentConstraintRow->m_relpos2CrossNormal;
				info2.rowskip = sizeof(btSolverConstraint)/sizeof(btScalar);//check this
				///the size of btSolverConstraint needs be a multiple of btScalar
				btAssert(info2.rowskip*sizeof(btScalar)== sizeof(btSolverConstraint));
				info2.m_constraintError = &currentConstraintRow->m_rhs;
				info2.cfm = &currentConstraintRow->m_cfm;
				info2.m_lowerLimit = &currentConstraintRow->m_lowerLimit;
				info2.m_upperLimit = &currentConstraintRow->m_upperLimit;
				constraints[i]->getInfo2(&info2);

				// Finalize the constraint setup
				//    All the solverConstraints in currentConstraintRow were recently initialized and assigned a pair of solverBodyIds (see approx. 30 lines up)
				//    Then compute the angular components, rhs, and applied impulse for each of those solver constraints
				//    Each constraint has its own row of solverConstraints
				for ( j=0; j<info1.m_numConstraintRows; j++ )
				{
					btSolverConstraint& solverConstraint = currentConstraintRow[j];

					{
						const btVector3& ftorqueAxis1 = solverConstraint.m_relpos1CrossNormal;
						solverConstraint.m_angularComponentA = constraint->getRigidBodyA().getInvInertiaTensorWorld() * ftorqueAxis1 * constraint->getRigidBodyA().getAngularFactor();
					}
					{
						const btVector3& ftorqueAxis2 = solverConstraint.m_relpos2CrossNormal;
						solverConstraint.m_angularComponentB = constraint->getRigidBodyB().getInvInertiaTensorWorld()*ftorqueAxis2*constraint->getRigidBodyB().getAngularFactor();
					}

					{
						btVector3 iMJlA = solverConstraint.m_contactNormal * rbA.getInvMass();
						btVector3 iMJaA = rbA.getInvInertiaTensorWorld() * solverConstraint.m_relpos1CrossNormal;
						btVector3 iMJlB = solverConstraint.m_contactNormal * rbB.getInvMass();//sign of normal?
						btVector3 iMJaB = rbB.getInvInertiaTensorWorld() * solverConstraint.m_relpos2CrossNormal;

						btScalar sum = iMJlA.dot(solverConstraint.m_contactNormal);
						sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
						sum += iMJlB.dot(solverConstraint.m_contactNormal);
						sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);

						solverConstraint.m_jacDiagABInv = btScalar(1.)/sum;
					}

					///fix rhs
					///todo: add force/torque accelerators
					{
						btScalar rel_vel;
						btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(rbA.getLinearVelocity()) + solverConstraint.m_relpos1CrossNormal.dot(rbA.getAngularVelocity());
						btScalar vel2Dotn = -solverConstraint.m_contactNormal.dot(rbB.getLinearVelocity()) + solverConstraint.m_relpos2CrossNormal.dot(rbB.getAngularVelocity());

						rel_vel = vel1Dotn+vel2Dotn;

						btScalar restitution = 0.f;
						btScalar positionalError = solverConstraint.m_rhs;//already filled in by getConstraintInfo2
						btScalar	velocityError = restitution - rel_vel;// * damping;
						btScalar	penetrationImpulse = positionalError*solverConstraint.m_jacDiagABInv;
						btScalar	velocityImpulse = velocityError *solverConstraint.m_jacDiagABInv;
						solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
						solverConstraint.m_appliedImpulse = 0.f;

					}
				}
			}
		}
	}
	// solverConstraints are now done being set up for btTypeConstraints

	return 1.f;

}  // solveBodyConstraints()






void shMolecularDynamicsConstraintSolver::initSolverBody( shSolverBody* solverBody, btCollisionObject* collisionObject )
{
	btRigidBody* rb = collisionObject ? btRigidBody::upcast(collisionObject) : 0;

	solverBody->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
	solverBody->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);

	if (rb)
	{
		//solverBody->m_invMass = rb->getInvMass();
		solverBody->m_originalBody = rb;
		solverBody->m_angularFactor = rb->getAngularFactor();

	}  // if
	else
	{
		//solverBody->m_invMass = 0.f;
		solverBody->m_originalBody = 0;
		//solverBody->m_angularFactor = 1.f;
		solverBody->m_angularFactor.setValue(1.f, 1.f, 1.f);

	}  // else

}  // initSolverBody()




int	shMolecularDynamicsConstraintSolver::getOrInitSolverBody( btCollisionObject& body )
{
	int solverBodyIdA = -1;

	if ( body.getCompanionId() >= 0 )
	{
		//body has already been converted
		solverBodyIdA = body.getCompanionId();

	}  // if
	else
	{
		btRigidBody* rb = btRigidBody::upcast( &body );
		if ( rb && rb->getInvMass() )
		{
			solverBodyIdA = m_tmpSolverBodyPool.size();
			shSolverBody& solverBody = m_tmpSolverBodyPool.expand();
			initSolverBody( &solverBody, &body );
			body.setCompanionId( solverBodyIdA );

		}  // if
		else
		{
			return 0;		//assume first one is a fixed solver body

		}  // else

	}  // else

	return solverBodyIdA;

}  // getOrInitSolverBody()




bool shMolecularDynamicsConstraintSolver::thereWasACollision() {

	if ( CollisionHappened != 0.f )
		return true;
	else
		return false;

}  // didAnythingCollide()



