/*
 * Copyright (c) 2011
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//
// Open Source Bullet Solver
//
#include "SIM_SolverBulletHolladay.h"

//#include "SIM_BulletData.h"
//#include "SIM_ConRelConeTwist.h"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <GEO/GEO_Quadric.h>
#include <PRM/PRM_Include.h>
#include <RBD/RBD_State.h>
#include <SIM/SIM_ActiveValue.h>
#include <SIM/SIM_ConAnchorObjSpacePos.h>
#include <SIM/SIM_ConAnchorObjSpaceRot.h>
#include <SIM/SIM_ConRelSpring.h>
#include <SIM/SIM_ConRelHard.h>
#include <SIM/SIM_ConstraintIterator.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_ForceGravity.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_GlueNetworkRelationship.h>
#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_SDF.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_TokenString.h>
#include <UT/UT_XformOrder.h>

#include <SIM/SIM_GuidePerObject.h>
#include <SIM/SIM_GuideTimeDep.h>
#include <SIM/SIM_PRMShared.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimCircle.h>
#include <GU/GU_PrimSphere.h>
#include <GU/GU_PrimTube.h>





// EDIT // This function is copied over from Bullet's btContactConstraint.cpp file (resolveSingleCollision()).
//  It computes the impulse of a collision.
//  However, unlike in btContactConstraint.cpp, it does not actually apply the impulses to the objects
btScalar resolveCollision(
        btRigidBody* body1,
        btCollisionObject* colObj2,
		const btVector3& contactPositionWorld,
		const btVector3& contactNormalOnB,
        const btContactSolverInfo& solverInfo,
		btScalar distance)
{
	btRigidBody* body2 = btRigidBody::upcast(colObj2);
    
	
    const btVector3& normal = contactNormalOnB;

    btVector3 rel_pos1 = contactPositionWorld - body1->getWorldTransform().getOrigin(); 
    btVector3 rel_pos2 = contactPositionWorld - colObj2->getWorldTransform().getOrigin();
    
    btVector3 vel1 = body1->getVelocityInLocalPoint(rel_pos1);
	btVector3 vel2 = body2? body2->getVelocityInLocalPoint(rel_pos2) : btVector3(0,0,0);
    btVector3 vel = vel1 - vel2;
    btScalar rel_vel;
    rel_vel = normal.dot(vel);
    
    btScalar combinedRestitution = body1->getRestitution() * colObj2->getRestitution();
    btScalar restitution = combinedRestitution* -rel_vel;
	
    btScalar positionalError = solverInfo.m_erp *-distance /solverInfo.m_timeStep ;
    btScalar velocityError = -(1.0f + restitution) * rel_vel;// * damping;
	btScalar denom0 = body1->computeImpulseDenominator(contactPositionWorld,normal);
	btScalar denom1 = body2? body2->computeImpulseDenominator(contactPositionWorld,normal) : 0.f;
	btScalar relaxation = 1.f;
	btScalar jacDiagABInv = relaxation/(denom0+denom1);
	
    btScalar penetrationImpulse = positionalError * jacDiagABInv;
    btScalar velocityImpulse = velocityError * jacDiagABInv;
	
    btScalar normalImpulse = penetrationImpulse+velocityImpulse;
	
    normalImpulse = 0.f > normalImpulse ? 0.f: normalImpulse;
	
    return normalImpulse;
}  // resolveCollision()
//////////






// automatically set the geometry representation parameters if requested
static void
simAutofit(SIM_Object &obj)
{
    // use const reference to check the autofit flag to avoid unnecessarily
    // unsharing the data
    const SIM_BulletData *bulletdata =
		SIM_DATA_GETCONST(obj, "Geometry/BulletData", SIM_BulletData);
    if(!bulletdata || !bulletdata->getAutofit())
	return;

    SIM_Geometry *geo = SIM_DATA_GET(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if(!geo)
	return;

    SIM_BulletData *data = SIM_DATA_GET(*geo, "BulletData", SIM_BulletData);
    if(!data)
	return;

    data->autofit(*geo);
}

//
// utility methods to translate between Houdini and Bullet math classes
//

inline static btVector3
simbtVector3(const UT_Vector3 &v)
{
    return btVector3(v.x(), v.y(), v.z());
}

inline static btQuaternion
simbtQuaternion(const UT_Quaternion &q)
{
    return btQuaternion(q.x(), q.y(), q.z(), q.w());
}

inline static btTransform
simbtTransform(const UT_Vector3 &t, const UT_Quaternion &r, const UT_Vector3 &pivot)
{
    return btTransform(simbtQuaternion(r),
		       simbtVector3(t + pivot - r.rotate(pivot)));
}

inline static UT_Vector3
utVector3(const btVector3 &v)
{
    return UT_Vector3(v.x(), v.y(), v.z());
}

inline static UT_Quaternion
utQuaternion(const btQuaternion &q)
{
    return UT_Quaternion(q.x(), q.y(), q.z(), q.w());
}

//
// basic methods missing in btMatrix3x3
//

// EDIT // Comment out operator+, which is now defined in bullet 2.78, the bullet version I am using
/*
inline static btMatrix3x3 operator+(const btMatrix3x3 &a, const btMatrix3x3 &b)
{
    btMatrix3x3 v;
    v[0] = a[0] + b[0];
    v[1] = a[1] + b[1];
    v[2] = a[2] + b[2];
    return v;
}

inline static const btMatrix3x3 &operator+=(btMatrix3x3 &a, const btMatrix3x3 &b)
{
    a[0] += b[0];
    a[1] += b[1];
    a[2] += b[2];
    return a;
}
*/


// calculates the center of mass and volume of a geometry described by a detail
static void
simCalculateCenterOfMass(const GU_Detail *gdp, btTransform *center_of_mass, fpreal *volume)
{
    // FIXME: this should choose a 'center_of_mass' transform where the local
    // inertial tensor is a diagonal matrix
    UT_Vector3 center(0, 0, 0);
    fpreal total_area = 0;

    const GEO_Primitive *prim;
    GA_FOR_ALL_PRIMITIVES(gdp, prim)
    {
	fpreal area = prim->calcArea();
	center += prim->baryCenter() * area;
	total_area += area;
    }

    if(total_area)
	center /= total_area;

    if(center_of_mass)
	*center_of_mass = btTransform(btQuaternion(0, 0, 0, 1), simbtVector3(center));

    if(volume)
    {
	UT_Vector3 p = center;
	*volume = 0;
	GA_FOR_ALL_PRIMITIVES(gdp, prim)
	    *volume += prim->calcVolume(p);
    }
}

// custom collision shape for concave objects
//
// This class owns all of the supporting data needed for the shape's
// representation.
class simbtGImpactMeshShape : public btGImpactMeshShape
{
    simbtGImpactMeshShape(int *indices, btVector3 *verts, btTriangleIndexVertexArray *mesh)
	: btGImpactMeshShape(mesh), myIndices(indices), myVertices(verts), myMesh(mesh) {}

public:
    ~simbtGImpactMeshShape()
    {
	delete myMesh;
	delete [] myVertices;
	delete [] myIndices;
    }

    // allocates a new btGImpactMeshShapes instance
    static simbtGImpactMeshShape *newbtGImpactMeshShape(
	const GU_Detail *gdp, const btTransform &center_of_mass)
    {
        // all data allocated here will be owned by the simbtGImpactMeshShape
	// instance
	int *indices = new int[gdp->primitives().entries()*3];
	int nindices = 0;
	const GEO_Primitive *prim;
	GA_FOR_ALL_PRIMITIVES(gdp, prim)
	{
	    if(prim->getPrimitiveId() & GEO_PrimTypeCompat::GEOPRIMPOLY)
	    {
                // only add triangles
		if(prim->getVertexCount() == 3)
                {
		    for(int i = 0; i < 3; ++i)
			indices[nindices++] = prim->getVertexElement(2-i).getPointIndex();
		}
	    }
	}

	if(!nindices)
	{
	    delete [] indices;
	    return 0;
	}

	btVector3 *verts = new btVector3[gdp->getNumPoints()];
	int nverts = 0;
	const btTransform &xform_inv = center_of_mass.inverse();
	for(GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
	    verts[nverts++] = xform_inv * simbtVector3(gdp->getPos3(*it));

	return new simbtGImpactMeshShape(indices, verts,
		    new btTriangleIndexVertexArray(nindices / 3, indices,
						   3 * sizeof(int), nverts,
						   (btScalar*)&verts[0].x(),
						   sizeof(btVector3))); 
    }

private:
    int *myIndices;
    btVector3 *myVertices;
    btTriangleIndexVertexArray *myMesh;
};

class simBulletGluedChunk;

// class used to describe the objects currently in the Bullet simulation
//
// This class owns all objects it allocates.  Destroying an instance of this
// class will remove its body from the Bullet simulation.
class simBulletObject
{
    enum ObjectState
    {
	OBJECT_STATE_UNUSED,
	OBJECT_STATE_SOLVING,
	OBJECT_STATE_AFFECTOR
    };

public:
    simBulletObject(int idx, btDiscreteDynamicsWorld *world)
	: myIndex(idx), myGluedChunk(0), myObjectState(OBJECT_STATE_UNUSED),
	  myDopId(-1), myActiveValue(false), myDynamicsWorld(world), myBody(0),
	  DidCollide(false)		// EDIT // Added DidCollide
	  
    {
    }

    ~simBulletObject()
    {
	destroyBody();
    }
	
	// EDIT // Code for saving out and accessing transforms in case a chunk's transform needs to be reset
	void saveTransforms()
	{
		SavedCenterOfMass = myCenterOfMass;
		if ( myBody )
		{
			SavedCenterOfMassTransform = myBody->getCenterOfMassTransform();
			SavedLinearVelocity = myBody->getLinearVelocity();
			SavedAngularVelocity = myBody->getAngularVelocity();
		}  // if
		setImpulses(0.0);
	}  // saveTransforms()
	
	void resetTransforms()
	{
		myCenterOfMass = SavedCenterOfMass;
		if ( myBody )
		{
			myBody->setCenterOfMassTransform( SavedCenterOfMassTransform );
			myBody->setLinearVelocity( SavedLinearVelocity );
			myBody->setAngularVelocity( SavedAngularVelocity );
		}  // if
	}  // resetTransforms()
	//////////
	
	// EDIT // Code for getting and setting whether a glued piece has collided
	bool getCollided() {return DidCollide;}
	void setCollided(bool dc) {DidCollide = dc;}
	//////////
	
    // get/set cached information about object's position in
    // simBulletState::myObjects
    int getIndex() const { return myIndex; }
    void setIndex(int idx) { myIndex = idx; }

    // get/set cached information about chunk containing this object
    simBulletGluedChunk *getGluedChunk() const { return myGluedChunk; }
    void setGluedChunk(simBulletGluedChunk *chunk) { myGluedChunk = chunk; }

    // get/set cached information about object's position in
    // simBulletGluedChunk::myPieces
    int getPieceIndex() const { return myPieceIndex; }
    void setPieceIndex(int idx) { myPieceIndex = idx; }

    // allows identification of unused objects
    bool isUnused() const { return myObjectState == OBJECT_STATE_UNUSED; }
    void setUnused() { myObjectState = OBJECT_STATE_UNUSED; }

    // allows identification of affectors
    bool isAffector() const { return myObjectState == OBJECT_STATE_AFFECTOR; }

    // DOP ID of corresponding SIM_Object
    int getDopId() const { return myDopId; }

    // raw Bullet body
    btRigidBody *getBody() const { return myBody; }

    // transform from the collision shape's local space to the SIM_Object's
    // local space
    const btTransform &getCenterOfMass() const { return myCenterOfMass; }

    // get/set cached information about impulses applied to resolve collisions
    fpreal getImpulses() const { return myImpulses; }
    void setImpulses(fpreal impulses) { myImpulses = impulses; }

    // updates Bullet simulation to reflect current state of the SIM_Object
    void updateFromObject(SIM_SolverBulletHolladay *solver, SIM_Object &obj,
			  bool affector_pass)
    {
        // update BulletData parameters
	simAutofit(obj);

        // indicate this object is part of the simulation
	if(affector_pass && myObjectState != OBJECT_STATE_SOLVING)
	    myObjectState = OBJECT_STATE_AFFECTOR;
	else
	    myObjectState = OBJECT_STATE_SOLVING;

        // force update of the body on the first invocation
	bool force = (myDopId < 0);
	if(force)
	    myDopId = obj.getObjectId();

	const SIM_BulletData *bulletdata =
		SIM_DATA_GETCONST(obj, "Geometry/BulletData", SIM_BulletData);
	const SIM_ActiveValue *activevalue =
		SIM_DATA_GETCONST(obj, "SolverParms/ActiveValue", SIM_ActiveValue);

	// update the Bullet geometry representation if necessary
	if(bulletdata && (force ||
		myBulletDataGuid != bulletdata->getUniqueId() ||
		((!activevalue || activevalue->getActive()) != myActiveValue)))
	{
	    // recreate the body
	    destroyBody();
	    myBulletDataGuid = bulletdata->getUniqueId();
		if (activevalue)			// EDIT // Debug crashing when the object has no activevalue
			myActiveValue = activevalue->getActive();
		else						// EDIT // "     "        "    "   "      "   "  "
			myActiveValue = false;	// EDIT // "     "        "    "   "      "   "  "
		
	    fpreal volume;
	    btCollisionShape *shape =
		    createCollisionShape(solver, obj, &myCenterOfMass, &volume);
		
	    if(shape)
	    {
		// update mass
		fpreal mass = 0;
		if(myObjectState == OBJECT_STATE_SOLVING)
		{
		    RBD_State *rbdstate = SIM_DATA_GET(obj, "Position", RBD_State);

		    if(rbdstate)
		    {
			if(rbdstate->getComputeMass())
			{
			    mass = rbdstate->getDensity() * volume;
			    rbdstate->setMass(mass);
			}
			else
			    mass = rbdstate->getMass();

			// allow user specified center of mass
			if(!rbdstate->getComputeCOM())
			{
			    btCompoundShape *offset_shape = new btCompoundShape();
			    const btVector3 &pivot = simbtVector3(rbdstate->getPivot());
			    btVector3 offset = btMatrix3x3(myCenterOfMass.getRotation()).transpose() * (myCenterOfMass.getOrigin() - pivot);
			    offset_shape->addChildShape(btTransform(btQuaternion(0, 0, 0, 1), offset), shape);
			    myCenterOfMass.setOrigin(pivot);
			    shape = offset_shape;
			}
		    }
		}
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(mass, inertia);

		// update position and material properties
		const SIM_Motion *motion = SIM_DATA_GETCONST(obj, "Position", SIM_Motion);
		btRigidBody::btRigidBodyConstructionInfo info = btRigidBody::btRigidBodyConstructionInfo(mass, 0, shape, inertia);

		const SIM_PhysicalParms *physicalparms = SIM_DATA_GETCONST(obj, "PhysicalParms", SIM_PhysicalParms);
		info.m_friction = physicalparms ? physicalparms->getFriction() : 0.2;
		info.m_restitution = physicalparms ? physicalparms->getBounce() : 0.5;

		// create new Bullet body
		myBody = new btRigidBody(info);
		myBody->setCenterOfMassTransform(motion ? simbtTransform(motion->getPosition(), motion->getOrientation(), motion->getPivot()) * myCenterOfMass : myCenterOfMass);
		if(bulletdata && !bulletdata->getDeactivate())
		    myBody->setActivationState(DISABLE_DEACTIVATION);
		if(!myGluedChunk)
		    myDynamicsWorld->addRigidBody(myBody);
	    }
	}

	if(myBody)
	{
	    // update the object's current position and velocity
	    const SIM_Motion *motion = SIM_DATA_GETCONST(obj, "Position", SIM_Motion);
	    if(motion)
	    {
		myBody->proceedToTransform(simbtTransform(motion->getPosition(), motion->getOrientation(), motion->getPivot()) * myCenterOfMass);
		UT_Vector3 vel = motion->getVelocity();
		UT_Vector3 avel = motion->getAngularVelocity();
		// inherit point velocities
		const RBD_State *rbdstate = SIM_DATA_GET(obj, "Position", RBD_State);
		if(rbdstate && rbdstate->getInheritVelocity())
		{
		    const SIM_Geometry *simgeo = obj.getGeometry();
		    if(simgeo)
		    {
			UT_Vector3 v, av;
			rigidMotionFromPointVelocities(v, av,
				utVector3(myBody->getCenterOfMassPosition()),
				simgeo, rbdstate);

			vel += v;
			avel += av;
		    }
		}
		myBody->setLinearVelocity(simbtVector3(vel));
		myBody->setAngularVelocity(simbtVector3(avel));
	    }
	}
    }

    // updates how Bullet handles collisions with this object
    void setCollisionFlags()
    {
	if(!myBody)
	    return;

	if(myObjectState != OBJECT_STATE_SOLVING)
	    myBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	else if(myBody->getCollisionFlags() == btCollisionObject::CF_KINEMATIC_OBJECT)
	{
	    // clear collision flags as this objects was previously treated as
	    // an affector
	    myBody->setCollisionFlags(0);
	}
    }

private:
    // destroy Bullet body
    void destroyBody()
    {
	if(!myBody)
	    return;

	if(!myGluedChunk)
	    myDynamicsWorld->removeRigidBody(myBody);
	destroyShape();
	delete myBody;
	myBody = 0;
    }

    // determine rigid motion from point velocities on simgeo
    void rigidMotionFromPointVelocities(UT_Vector3 &vel, UT_Vector3 &avel,
					const UT_Vector3 &com,
					const SIM_Geometry *simgeo,
					const RBD_State *rbdstate)
    {
	UT_Vector3Array posarray, velarray;
	GU_DetailHandleAutoReadLock gdl(simgeo->getGeometry());
	const GU_Detail *gdp = gdl.getGdp();

	GA_ROAttributeRef voff = gdp->findVelocityAttribute(GEO_POINT_DICT);
	// Valid velocity attribute.
	if(!voff.isValid())
	{
	    avel = vel = UT_Vector3(0, 0, 0);
	    return;
	}

	UT_DMatrix4 toworld;
	rbdstate->getTransform(toworld);

	UT_DMatrix4 geoxform;
	simgeo->getTransform(geoxform);

	toworld = geoxform * toworld;

	UT_Matrix3 toworld_vec;
	toworld_vec = toworld;

	const GA_IndexMap &map = gdp->getPointMap();
	for(GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
	{
	    GEO_Point pt(map, *it);
	    UT_Vector3 pos = pt.getPos();
	    UT_Vector3 v = pt.getValue<UT_Vector3>(voff);

	    // Include the transform of the object.
	    pos *= toworld;
	    v *= toworld_vec;

	    posarray.append(pos);
	    velarray.append(v);
	}

	// Get the corresponding rigid motion.
	RBD_State::projectRigidMotion(vel, avel, com, posarray, velarray);
    }

    // create collision shape for convex geometry
    btCollisionShape *
    generateConvexObject(SIM_SolverBulletHolladay *solver, SIM_Object &obj,
			 GU_Detail *gdp, const SIM_BulletData *bulletdata,
			 btTransform *center_of_mass, fpreal *volume)
    {
	btTransform xform;
	simCalculateCenterOfMass(gdp, &xform, volume);
	if(center_of_mass)
	    *center_of_mass = xform;

	btCompoundShape *comp = new btCompoundShape();
	btTransform xform_inv = xform.inverse();

	GA_PointGroup *unused_points = gdp->newInternalPointGroup();
	unused_points->toggleEntries();

	// use standard collision shapes for spheres and tubes if possible
	for(int i = gdp->primitives().entries() - 1; i >= 0; --i)
	{
	    const GEO_Primitive *prim = gdp->primitives()(i);
	    int prim_id = prim->getTypeId().get();

	    if(prim_id == GEO_PRIMSPHERE || prim_id == GEO_PRIMTUBE)
	    {
		const GEO_Quadric *quadric = static_cast<const GEO_Quadric*>(prim);

		UT_Vector3 translate = prim->baryCenter();

		UT_Matrix3D xform = quadric->getTransform();
		UT_Vector3 scale;
		xform.extractScales(scale);

		UT_Quaternion rot_q;
		rot_q.updateFromRotationMatrix(xform);

		btTransform bt_xform(simbtQuaternion(rot_q), simbtVector3(translate));

		if(prim_id == GEO_PRIMSPHERE)
		{
		    btSphereShape *shape = new btSphereShape(1);
		    shape->setLocalScaling(simbtVector3(scale));
		    shape->setUserPointer(this);
		    comp->addChildShape(xform_inv * bt_xform, shape);
		}
		else
		{
		    scale.y() /= 2;
		    btCylinderShape *shape = new btCylinderShape(simbtVector3(scale));
		    shape->setUserPointer(this);
		    comp->addChildShape(xform_inv * bt_xform, shape);
		}

		// keep the point that represent this primitive, so we can skip
		// it later
		const GEO_Vertex vertex = prim->getVertexElement(0);
		unused_points->removeOffset(vertex.getPointOffset());
	    }
	}
	if(gdp->getNumPoints() > 0)
	{
	    // put all the remaining points into a convex hull shape
	    btConvexHullShape *shape = new btConvexHullShape();
	    shape->setUserPointer(this);
	    fpreal margin = bulletdata->getCollisionMargin();
	    if(bulletdata->getOptimized())
	    {
		GU_Detail adjusted_gdp;
		GU_Detail *adj_gdp;
		if(bulletdata->getAdjustGeometry())
		{
		    adj_gdp = &adjusted_gdp;
		    margin *= bulletdata->getAdjustFactor();

		    // shrink geometry by 'margin'
		    if(!gdp->tetrahedralizeForShrink(unused_points, adj_gdp))
		    {
			solver->addError(&obj, SIM_MESSAGE,
			    "Unable to convert geometry into a convex hull.",
			    UT_ERROR_WARNING);
		    }

		    adj_gdp->shrink(-margin);

		    if(!adj_gdp->getNumPoints())
		    {
			// FIXME: this should only consider 'unused_points'
			btTransform center;
			simCalculateCenterOfMass(gdp, &center, 0);
			adj_gdp->setPos3(adj_gdp->appendPointOffset(),
					 utVector3(center.getOrigin()));
		    }
		}
		else
		    adj_gdp = gdp;

		for(GA_Iterator it(adj_gdp->getPointRange()); !it.atEnd(); ++it)
		    shape->addPoint(xform_inv * simbtVector3(adj_gdp->getPos3(*it)));
	    }
	    else
	    {
		btAlignedObjectArray<btVector3> vertices;

		for(GA_Iterator it(gdp->getPointRange(unused_points)); !it.atEnd(); ++it)
		    vertices.push_back(xform_inv * simbtVector3(gdp->getPos3(*it)));
		int n = vertices.size();
		if(n > 0)
		{
		    if(bulletdata->getAdjustGeometry())
		    {
			margin *= bulletdata->getAdjustFactor();

			btAlignedObjectArray<btVector3> planes;

			// Shrink the object by collision margin to eliminate gap
			btGeometryUtil::getPlaneEquationsFromVertices(vertices,
								      planes);
			for(int i = planes.size() - 1; i >= 0; --i)
			    planes[i][3] += margin;

			btAlignedObjectArray<btVector3> shifted_vx;
			btGeometryUtil::getVerticesFromPlaneEquations(planes,
								shifted_vx);
			// Verify that the convex hull is still valid
			int sn = shifted_vx.size();
			if(sn > 0)
			{
			    for (int i = 0; i < sn; ++i)
				shape->addPoint(shifted_vx[i]);
			}
			else
			{
			    solver->addError(&obj, SIM_MESSAGE,
				"Object is too small to be adjusted for collision"
				"margin, Using original geometry instead.",
				UT_ERROR_WARNING);
			    for (int i = 0; i < n; ++i)
				shape->addPoint(vertices[i]);
			}
		    }
		    else
		    {
			for(int i = 0; i < n; ++i)
			    shape->addPoint(vertices[i]);
		    }
		}
	    }
	    shape->setMargin(margin);
	    comp->addChildShape(btTransform::getIdentity(), shape);
	}
	gdp->destroyPointGroup(unused_points);
	return comp;
    }

    // create collision shape for an ODE style compound object
    btCollisionShape *
    generateCompoundObject(SIM_SolverBulletHolladay *solver, SIM_Object &obj,
	const GU_Detail *gdp, const SIM_BulletData *bulletdata,
	btTransform *center_of_mass, fpreal *volume)
    {
	btTransform xform;
	simCalculateCenterOfMass(gdp, &xform, volume);
	if(center_of_mass)
	    *center_of_mass = xform;

	btCompoundShape *comp = new btCompoundShape();
	btTransform xform_inv = xform.inverse();

	// this uses the same method as ODE to get the primitive data.
	const GA_PointGroup *point_group = gdp->findPointGroup("odePointGroup");

	if(!point_group)
	{
	    solver->addError(&obj, SIM_MESSAGE,
			"Required point group \"odePointGroup\" from node"
			" \"Bake ODE\" not found",
			UT_ERROR_WARNING);
	}
	else if(point_group->entries())
	{
	    //Collect some attrs off the point, these define the ODE body
	    // FIXME: should this use the primPosition attribute?
	    GA_ROAttributeRef protation = gdp->findFloatTuple(GA_ATTRIB_POINT, "primRotation", 3, 3);
	    GA_ROAttributeRef pscale = gdp->findFloatTuple(GA_ATTRIB_POINT, "primScale", 3, 3);
	    GA_ROAttributeRef plength = gdp->findFloatTuple(GA_ATTRIB_POINT, "primLength", 1);
	    GA_ROAttributeRef pradius = gdp->findFloatTuple(GA_ATTRIB_POINT, "primRadius", 1);
	    GA_ROAttributeRef ptype = gdp->findIntTuple(GA_ATTRIB_POINT, "primType", 1);

	    for(GA_Iterator it(gdp->getPointRange(point_group)); !it.atEnd(); ++it)
	    {
		const GEO_Point ppt(gdp->getPointMap(), *it);

		btCollisionShape *shape;
		switch(ptype.isValid() ? ppt.getValue<int>(ptype) : 0)
		{
		    case 0: // Box
			{
			    UT_Vector3 scale = pscale.isValid() ? ppt.getValue<UT_Vector3>(pscale) : UT_Vector3(1, 1, 1);
			    shape = new btBoxShape(simbtVector3(0.5 * scale));
			}
			break;
		    case 1: // Sphere
			{
			    fpreal radius = pradius.isValid() ? ppt.getValue<float>(pradius) : 1.0;
			    shape = new btSphereShape(radius);
			}
			break;
		    case 2: // Cylinder
			{
			    fpreal radius = pradius.isValid() ? ppt.getValue<float>(pradius) : 1.0;
			    fpreal length = plength.isValid() ? ppt.getValue<float>(plength) : 1.0;
			    shape = new btCylinderShapeZ(btVector3(radius, radius, length/2));
			}
			break;
		    case 3: // Capsule
			{
			    fpreal radius = pradius.isValid() ? ppt.getValue<float>(pradius) : 1.0;
			    fpreal length = plength.isValid() ? ppt.getValue<float>(plength) : 1.0;
			    shape = new btCapsuleShapeZ(radius, length);
			}
			break;

		    default:
			shape = 0;
			break;
		}

		if(shape)
		{
		    shape->setUserPointer(this);

		    UT_Vector3 pos = ppt.getPos3();
		    UT_Vector3 rot = protation.isValid() ? ppt.getValue<UT_Vector3>(protation) : UT_Vector3(0, 0, 0);
		    btQuaternion bt_rot;
		    // This appears backwards.  This is because bullet
		    // actually takes its parameters backwards in ZYX
		    // mode.  Probably should just abandon using bullet
		    // and use our own quaternion methods.
		    bt_rot.setEulerZYX(rot.z(), rot.y(), rot.x());
		    comp->addChildShape(xform_inv * btTransform(bt_rot, simbtVector3(pos)), shape);
		}
	    }
	}

	return comp;
    }

    // create a collision shape for the SIM_Object
    // 
    // the 'user pointer' of all leaf shapes should point to this object so we
    // can map collision impacts back to this object
    btCollisionShape *
    createCollisionShape(SIM_SolverBulletHolladay *solver, SIM_Object &obj,
			 btTransform *center_of_mass, fpreal *volume)
    {
	// get the geometry from the proxy data if it exists, otherwise from
	// the Geometry subdata
	const SIM_Geometry *simgeo =
		SIM_DATA_GETCONST(obj, "ODE_Composite_Body", SIM_Geometry);
	
	if(!simgeo)
	    simgeo = obj.getGeometry();
	
	const SIM_BulletData *bulletdata =
		SIM_DATA_GETCONST(obj, "Geometry/BulletData", SIM_BulletData);
	
	if(!simgeo || !bulletdata)
	    return 0;
	
	const SIM_SDF *sdf = SIM_DATA_GETCONST(*simgeo, SIM_SDF_DATANAME, SIM_SDF);
	if(sdf && sdf->getMode() == 4)
	{
	    // ground plane representation
	    if(center_of_mass)
		*center_of_mass = btTransform::getIdentity();
	    if(volume)
		*volume = 0;

	    btCollisionShape *shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
	    shape->setUserPointer(this);
	    return shape;
	}
	
	UT_String geoRep;
	bulletdata->getGeoRep(geoRep);

	if(geoRep == GEO_REP_BOX)
	{
	    // box representation
	    btVector3 size = simbtVector3(bulletdata->getPrimSD());
	    if(center_of_mass)
	    {
		UT_XformOrder order;
		order.mainOrder(UT_XformOrder::RTS);
		order.rotOrder(UT_XformOrder::XYZ);
		*center_of_mass = btTransform(simbtQuaternion(UT_Quaternion(M_PI / 180 * bulletdata->getPrimRD(), order)), simbtVector3(bulletdata->getPrimTD()));
	    }
	    if(volume)
		*volume = 8 * size.x() * size.y() * size.z();

	    btCollisionShape *shape = new btBoxShape(size);
	    shape->setUserPointer(this);
	    return shape;
	}
	if(geoRep == GEO_REP_SPHERE)
	{
	    // sphere representation
	    fpreal radius = bulletdata->getPrimRadius();

	    if (center_of_mass)
		*center_of_mass = btTransform(btQuaternion(0, 0, 0, 1), simbtVector3(bulletdata->getPrimTD()));
	    if (volume)
		*volume = (4.0/3.0) * M_PI * radius * radius * radius;

	    btCollisionShape *shape = new btSphereShape(radius);
	    shape->setUserPointer(this);
	    return shape;
	}
	if(geoRep == GEO_REP_CAPSULE)
	{
	    // capsule representation
	    fpreal length = bulletdata->getPrimLength();
	    fpreal radius = bulletdata->getPrimRadius();

	    if(center_of_mass)
	    {
		UT_XformOrder order;
		order.mainOrder(UT_XformOrder::RTS);
		order.rotOrder(UT_XformOrder::XYZ);
		*center_of_mass = btTransform(simbtQuaternion(UT_Quaternion(M_PI / 180 * bulletdata->getPrimRD(), order)), simbtVector3(bulletdata->getPrimTD()));
	    }
	    if(volume)
		*volume = M_PI * radius * radius * (4.0/3.0 * radius + length);

	    btCollisionShape *shape = new btCapsuleShape(radius, length);
	    shape->setUserPointer(this);
	    return shape;
	}
	
	// bake transform into the detail
	GU_DetailHandleAutoReadLock gdl(simgeo->getGeometry());
	GU_Detail sim_gdp(gdl.getGdp());
	UT_Matrix4D geoXForm;
	simgeo->getTransform(geoXForm);
	UT_Matrix4 geo_xform(geoXForm);
	sim_gdp.transform(geo_xform);
	
	if(geoRep == GEO_REP_AS_IS)
	{
	    // triangulate the transformed gdp
	    GU_Detail mod_gdp(&sim_gdp);
	    mod_gdp.convex(3);
	    if(bulletdata->getGeoConvex())
	    {
		// treat as convex hull
		return generateConvexObject(solver, obj, &mod_gdp, bulletdata,
					    center_of_mass, volume);
	    }

	    // treat as concave object
	    btTransform xform;
	    simCalculateCenterOfMass(&mod_gdp, &xform, volume);
	    if(center_of_mass)
		*center_of_mass = xform;

	    simbtGImpactMeshShape *shape =
		simbtGImpactMeshShape::newbtGImpactMeshShape(&mod_gdp, xform);
	    if(shape)
	    {
		shape->setUserPointer(this);
		shape->updateBound();
		btGImpactCollisionAlgorithm::registerAlgorithm(static_cast<btCollisionDispatcher *>(myDynamicsWorld->getDispatcher()));
	    }
	    return shape;
	}
	if(geoRep == GEO_REP_COMPOUND)
	{
	    // Do not need to triangulate as we are dealing
	    // with the underlying points.
	    return generateCompoundObject(solver, obj, &sim_gdp, bulletdata,
					  center_of_mass, volume);
	}
	return 0;
    }

    // destroy this object's collision shape
    void destroyShape()
    {
	if(!myBody)
	    return;

	// use a work list to destroy the tree of collision shapes
	UT_PtrArray<btCollisionShape *> work;
	work.append(myBody->getCollisionShape());
	while(work.entries())
	{
	    btCollisionShape *shape = work.removeLast();
	    if(shape->isCompound())
	    {
		// add all child shapes to the work list
		btCompoundShape *comp = static_cast<btCompoundShape *>(shape);
		for(int i = comp->getNumChildShapes() - 1; i >= 0; --i)
		    work.append(comp->getChildShape(i));
	    }
	    delete shape;
	}
    }

    // current index of this instance in the simBulletState::myObjects array
    int myIndex;

    // chunk to which this object is glued or NULL
    simBulletGluedChunk *myGluedChunk;

    // current index of this object in the simBulletGluedChunk::myPieces array
    int myPieceIndex;

    // indicates what type of object this represent (unused, solving, or
    // affector)
    ObjectState myObjectState;

    // DOP ID of SIM_Object this object represents
    int myDopId;

    // GUID of SIM_BulletData used to create Bullet representation
    UT_Guid myBulletDataGuid;
    bool myActiveValue;

    // the Bullet simulation
    btDiscreteDynamicsWorld *myDynamicsWorld;

    // the raw Bullet body owned by this object
    btRigidBody *myBody;

    // transform from the collision shape's local space to the SIM_Object's
    // local space
    btTransform myCenterOfMass;

    // impulses applied to resolve collisions
    fpreal myImpulses;
	
	// EDIT // Variables for saving out the object's transforms
	btVector3 SavedLinearVelocity;
	btVector3 SavedAngularVelocity;
	btTransform SavedCenterOfMass;
	btTransform SavedCenterOfMassTransform;
	btVector3 SavedCenterOfMassPosition;
	bool DidCollide;
	//////////
};

// class used to describe the glued objects currently in the Bullet simulation
//
// This class owns only the objects it allocates.  Destroying an instance of
// this class will remove its body from the Bullet simulation.
class simBulletGluedChunk
{
public:
    simBulletGluedChunk(int idx, btDiscreteDynamicsWorld *world)
	: myIndex(idx), myDynamicsWorld(world), myBody(0)
    {
    }

    ~simBulletGluedChunk()
    {
	for(int i = myPieces.entries() - 1; i >= 0; --i)
	    removePiece(i);
	destroyBody();
    }
	
	// EDIT // Functions to get and set the saved linear and angular velocity of this glued chunk
	void saveTransforms()
	{
		SavedLinearVelocity =  myBody->getLinearVelocity();
		SavedAngularVelocity = myBody->getAngularVelocity();
		SavedCenterOfMassTransform = myBody->getCenterOfMassTransform();
		SavedCenterOfMassPosition = myBody->getCenterOfMassPosition();
		SavedCenterOfMass = myCenterOfMass;
		
		int numPieces = myPieces.entries();
		for ( int i = 0; i < numPieces; i++ )
		{
			simBulletObject* curPiece = myPieces(i);
			if ( curPiece )
				curPiece->saveTransforms();
		}  // for i
	}  // saveTransforms()
	
	void resetTransforms()
	{
		myBody->setLinearVelocity( SavedLinearVelocity );
		myBody->setAngularVelocity( SavedAngularVelocity );
		myBody->setCenterOfMassTransform( SavedCenterOfMassTransform );
		myCenterOfMass = SavedCenterOfMass;
		
		int numPieces = myPieces.entries();
		for ( int i = 0; i < numPieces; i++ )
		{
			simBulletObject* curPiece = myPieces(i);
			if ( curPiece )
				curPiece->resetTransforms();
		}  // for i
	}  // resetTransforms()
	//////////
	
    // get/set cached information about the chunks's position in the
    // simBulletState::myGluedChunks array
    int getIndex() const { return myIndex; }
    void setIndex(int idx) { myIndex = idx; }

    // methods to inspect pieces of this chunk
    int getNumPieces() const { return myPieces.entries(); }
    simBulletObject *getPiece(int i) const { return myPieces(i); }

    // transform from the collision shape's local space to the chunk's local
    // space
    const btTransform &getCenterOfMass() const { return myCenterOfMass; }
	
    // raw Bullet body
    btRigidBody *getBody() const { return myBody; }

    // added a piece to this chunk
    void addPiece(simBulletObject *obj)
    {
	UT_ASSERT(!obj->getGluedChunk());
	obj->setGluedChunk(this);
	obj->setPieceIndex(myPieces.entries());
	myPieces.append(obj);
	// remove original body from the simulation as the chunk's body will
	// now represent it
	btRigidBody *body = obj->getBody();
	if(body)
	    myDynamicsWorld->removeRigidBody(body);
    }

    // removes a piece from this chunk
    void removePiece(int idx)
    {
	simBulletObject *obj = myPieces(idx);
	// insert the original body into the simulation as the chunk's body
	// will no longer represent it
	btRigidBody *body = obj->getBody();
	if(body)
	    myDynamicsWorld->addRigidBody(body);
	obj->setGluedChunk(0);
	// shuffle pieces to fill hole created by removing 'obj'
	simBulletObject *temp = myPieces.removeLast();
	if(obj != temp)
	{
	    temp->setPieceIndex(idx);
	    myPieces(idx) = temp;
	}
    }

    // create a Bullet body to represent the chunk
    void updateBody(const SIM_Engine &engine)
    {
	destroyBody();

	bool contains_affector = false;
	int n = myPieces.entries();

	// if we have an animated affector, apply the animation to the the
	// other glued pieces
	for(int i = 0; i < n; ++i)
	{
	    simBulletObject *obj = myPieces(i);
	    if(obj->isAffector())
	    {
		const SIM_Object *curobj = engine.getSimulationObjectFromId(obj->getDopId());
		if(!curobj)
		    continue;
		
		const SIM_Time oldtime = engine.getSimulationTime() - engine.getTimeStep();
		const SIM_Object *oldobj = engine.getObjectAtTime(*curobj, oldtime, false);
		if(!oldobj)
		    continue;

		// transform non-affectors
		UT_DMatrix4 old_xform;
		SIMgetGeometryTransform(old_xform, *oldobj);

		UT_DMatrix4 new_xform;
		SIMgetGeometryTransform(new_xform, *curobj);

		old_xform.invert();
		old_xform *= new_xform;

		UT_DMatrix3 rot;
		old_xform.extractRotate(rot);
		UT_Quaternion rot_q;
		rot_q.updateFromRotationMatrix(rot);

		UT_Vector3 orig;
		old_xform.getTranslates(orig);

		btTransform xform(simbtQuaternion(rot_q), simbtVector3(orig));

		for(int j = 0; j < n; ++j)
		{
		    simBulletObject *obj2 = myPieces(j);
		    btRigidBody *body = obj2->getBody();
		    if(!body || obj2->isAffector())
			continue;

		    body->proceedToTransform(
				xform * body->getCenterOfMassTransform());
		}

		contains_affector = true;
		break;
	    }
	}

	// average properties of child pieces
	btVector3 center_of_mass(0, 0, 0);
	fpreal total_mass = 0;
	btVector3 vel(0, 0, 0);
	btVector3 angular_momentum(0, 0, 0);
	fpreal friction = 0;
	fpreal restitution = 0;

	for(int i = 0; i < n; ++i)
	{
	    simBulletObject *obj = myPieces(i);
	    btRigidBody *body = obj->getBody();
	    if(!body)
		continue;

	    fpreal mass = body->getInvMass();
	    if(!mass)
		continue;

	    mass = 1.0 / mass;
	    total_mass += mass;
	    const btVector3 &pos = body->getCenterOfMassPosition();
	    center_of_mass += mass * pos;
	    const btVector3 &p = mass * body->getLinearVelocity();
	    vel += p;
	    angular_momentum += body->getInvInertiaTensorWorld().inverse() * body->getAngularVelocity() + pos.cross(p);
	    friction += mass * body->getFriction();
	    restitution += mass * body->getRestitution();
	}
	btVector3 avel(0, 0, 0);
	if(total_mass)
	{
	    // finish computation
	    center_of_mass /= total_mass;
	    angular_momentum -= center_of_mass.cross(vel);
	    vel /= total_mass;
	    friction /= total_mass;
	    restitution /= total_mass;

	    // find angular velocity
	    if(!angular_momentum.isZero())
	    {
		// construct Q such that: Q * <1, 0, 0> = axis_of_rotation
		UT_Quaternion rot;
		rot.updateFromVectors(UT_Vector3(1, 0, 0), utVector3(angular_momentum.normalized()));
		btMatrix3x3 Q(simbtQuaternion(rot));
		btMatrix3x3 inv_Q(Q.transpose());

		// accumulate rotational inertia of each piece
		btMatrix3x3 rotational_inertia(0, 0, 0, 0, 0, 0, 0, 0, 0);
		for(int i = 0; i < n; ++i)
		{
		    simBulletObject *obj = myPieces(i);
		    btRigidBody *body = obj->getBody();
		    if(!body)
			continue;

		    fpreal mass = body->getInvMass();
		    if(!mass)
			continue;

		    mass = 1 / mass;
		    const btVector3 &delta = inv_Q * (body->getCenterOfMassPosition() - center_of_mass);

		    // use parallel-axis theorem
		    rotational_inertia += body->getInvInertiaTensorWorld().inverse() + Q * btMatrix3x3(mass * (delta.y() * delta.y() + delta.z() * delta.z()), 0, 0, 0, 0, 0, 0, 0, 0) * inv_Q;
		}
		avel = rotational_inertia.inverse() * angular_momentum;
	    }
	}

	// FIXME: we should provide a better way of calculating the
	// center_of_mass transform
	myCenterOfMass = btTransform(btQuaternion(0, 0, 0, 1), center_of_mass);
	const btTransform &com_inv = myCenterOfMass.inverse();

	// our collision shape is the union of the collision shapes from the
	// individual pieces
	btCompoundShape *shape = new btCompoundShape();
	for(int i = 0; i < n; ++i)
	{
	    btRigidBody *body = myPieces(i)->getBody();
	    if(body)
	    {
		// only add the leaf collision shapes of each piece so we can
		// identify the shapes in the collision response callback
		addLeafShapes(shape, com_inv * body->getCenterOfMassTransform(),
			      body->getCollisionShape());
	    }
	}

	// calculate inertial tensor
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(total_mass, inertia);

	// create now Bullet body for the chunk
	btRigidBody::btRigidBodyConstructionInfo info = btRigidBody::btRigidBodyConstructionInfo(total_mass, 0, shape, inertia);
	info.m_friction = friction;
	info.m_restitution = restitution;

	myBody = new btRigidBody(info);
	myBody->setCenterOfMassTransform(myCenterOfMass);
	myBody->setLinearVelocity(vel);
	myBody->setAngularVelocity(avel);

	if(contains_affector)
	    myBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

	myDynamicsWorld->addRigidBody(myBody);
    }

private:
    // destroy BulletBody
    void destroyBody()
    {
	if(!myBody)
	    return;

	myDynamicsWorld->removeRigidBody(myBody);
	delete myBody->getCollisionShape();
	delete myBody;
	myBody = 0;
    }

    // add all leaf collision shapes from a tree to 'parent'
    void addLeafShapes(btCompoundShape *parent,
	const btTransform &xform, btCollisionShape *shape)
    {
	if(shape->isCompound())
	{
	    btCompoundShape *comp = static_cast<btCompoundShape *>(shape);
	    for(int i = comp->getNumChildShapes() - 1; i >= 0; --i)
	    {
		addLeafShapes(parent, xform * comp->getChildTransform(i),
			      comp->getChildShape(i));
	    }
	}
	else
	    parent->addChildShape(xform, shape);
    }

    // current index of this instance in the simBulletState::myGluedChunks array
    int myIndex;

    // list of objects glued together
    UT_PtrArray<simBulletObject *> myPieces;

    // the Bullet simulation
    btDiscreteDynamicsWorld *myDynamicsWorld;

    // raw Bullet body
    btRigidBody *myBody;

    // transform from the collision shape's local space to the chunk's local space
    btTransform myCenterOfMass;
	
	// EDIT // Variables for saving out the chunk's transforms
	btVector3 SavedLinearVelocity;
	btVector3 SavedAngularVelocity;
	btTransform SavedCenterOfMass;
	btTransform SavedCenterOfMassTransform;
	btVector3 SavedCenterOfMassPosition;
	//////////
};

// map collision response to the involved simBulletObject
static simBulletObject *
simObjectFromShapeIdx(int idx, btRigidBody *body)
{
    if(idx < 0)
	idx = 0;

    btCollisionShape *shape = body->getCollisionShape();
    while(shape->isCompound())
    {
	btCompoundShape *comp = static_cast<btCompoundShape *>(shape);
	shape = comp->getChildShape(idx < comp->getNumChildShapes() ? idx : 0);
    }
    // the user pointer of every leaf shape should point to the owning
    // simBulletObject
    return static_cast<simBulletObject *>(shape->getUserPointer());
}

// map collision response to the involved simBulletObject
static simBulletObject *
simObjectFromShapeIdxGetFirstChild(int idx, btRigidBody *body)
{
    if(idx < 0)
	idx = 0;

    btCollisionShape *shape = body->getCollisionShape();
    if(shape->isCompound())
    {
	btCompoundShape *comp = static_cast<btCompoundShape *>(shape);
	shape = comp->getChildShape(idx < comp->getNumChildShapes() ? idx : 0);
    }
    // the user pointer of every leaf shape should point to the owning
    // simBulletObject
    return static_cast<simBulletObject *>(shape->getUserPointer());
}

// describes a network of directed edges with 'pr', 'pt0', and 'pt1' properties
class simGlueNetwork
{
public:
    simGlueNetwork(SIM_GlueNetworkRelationship *rel=0) : myRelationship(rel)
    {
    }

    // record a directed edge from 'idx0' to 'idx1'
    void addEdge(int idx0, int idx1)
    {
	myIndex0.append(idx0);
	myIndex1.append(idx1);
    }

    // record an edge from 'idx0' to 'idx1' and the source primitive and points
    // from myRelationship
    void addRelEdge(int idx0, int idx1, GA_Index pr, GA_Index pt0, GA_Index pt1)
    {
	UT_ASSERT(myIndex0.entries() == myPrimitiveNumber.entries());

	addEdge(idx0, idx1);
	myPrimitiveNumber.append(pr);
	myPoint0.append(pt0);
	myPoint1.append(pt1);
    }

    // accessor methods
    SIM_GlueNetworkRelationship *getRelationship() const { return myRelationship; }
    int getNumEdges() const { return myIndex0.entries(); }
    int getIndex0(int i) const { return myIndex0(i); }
    int getIndex1(int i) const { return myIndex1(i); }
    GA_Index getPrimitiveNumber(int i) const { return myPrimitiveNumber(i); }
    GA_Index getPoint0(int i) const { return myPoint0(i); }
    GA_Index getPoint1(int i) const { return myPoint1(i); }
	
private:
    // source SIM_GlueNetworkRelationship or NULL
    SIM_GlueNetworkRelationship *myRelationship;

    // start node for directed edge
    UT_IntArray myIndex0;

    // end node for directed edge
    UT_IntArray myIndex1;

    // this edge's source primitive from the SIM_GlueNetworkRelationship
    GA_IndexArray myPrimitiveNumber;

    // start node's source point from the SIM_GlueNetworkRelationship
    GA_IndexArray myPoint0;

    // end node's source point from the SIM_GlueNetworkRelationship
    GA_IndexArray myPoint1;
};

// describes constraints in the Bullet simulation
class simBulletConstraint
{
public:
    simBulletConstraint(SIM_ConRel *rel, btDiscreteDynamicsWorld *world,
	simBulletObject *objA, const btTransform &transA,
	simBulletObject *objB, const btTransform &transB,
	const btVector3 &velB, bool rotational)
	: myRel(rel), myDynamicsWorld(world), myConstraint(0), myBody(0)
    {
	// get anchor A
	simBulletGluedChunk *chunkA = objA->getGluedChunk();
	btRigidBody *bodyA = chunkA ? chunkA->getBody() : objA->getBody();
	if(!bodyA)
	    return;

	// get anchor B
	btRigidBody *bodyB;
	if(objB)
	{
	    simBulletGluedChunk *chunkB = objB->getGluedChunk();
	    bodyB = chunkB ? chunkB->getBody() : objB->getBody();
	    if(!bodyB)
		return;
	}
	else
	{
	    // create a dummy body for the world space anchor
	    myBody = new btRigidBody(0.0, 0, new btCompoundShape(), btVector3(0, 0, 0));
	    myBody->setCenterOfMassTransform(transB);
	    myDynamicsWorld->addRigidBody(myBody);
	    myBody->setActivationState(DISABLE_SIMULATION);
	    if(rotational)
		myBody->setAngularVelocity(velB);
	    else
		myBody->setLinearVelocity(velB);
	    bodyB = myBody;
	}

	btTransform localA = bodyA->getCenterOfMassTransform().inverse() * transA;
	btTransform localB = bodyB->getCenterOfMassTransform().inverse() * transB;

	// create constraint
	const SIM_ConRelSpring *spring = SIM_DATA_CASTCONST(rel, SIM_ConRelSpring);
	if(spring)
	{
	    // spring constraint
	    btGeneric6DofSpringConstraint *c = new btGeneric6DofSpringConstraint(*bodyA, *bodyB, localA, localB, false);
	    myConstraint = c;

	    // lower > upper, no limit
	    c->setAngularLowerLimit(btVector3(-1, -1, -1));
	    c->setAngularUpperLimit(btVector3(1, 1, 1));
	    c->setLinearLowerLimit(btVector3(1, 1, 1));
	    c->setLinearUpperLimit(btVector3(-1, -1, -1));

	    fpreal strength = spring->getStrength();
	    fpreal damping = SYSmax(1 - spring->getDamping(), 0.0);
	    int idx = rotational ? 3 : 0;
	    for(int i = 0; i < 3; ++i, ++idx)
	    {
		c->enableSpring(idx, true);
		c->setStiffness(idx, strength);
		c->setDamping(idx, damping);
		c->setEquilibriumPoint(idx, 0.0f);
	    }
	}
	else if(SIM_DATA_CASTCONST(rel, SIM_ConRelHard))
	{
	    // hard constraint
	    btGeneric6DofConstraint *c = new btGeneric6DofConstraint(*bodyA, *bodyB, localA, localB, false);
	    myConstraint = c;

	    if(rotational)
	    {
		c->setLinearLowerLimit(btVector3(1, 1, 1));
		c->setLinearUpperLimit(btVector3(-1, -1, -1));
		c->setAngularLowerLimit(btVector3(0, 0, 0));
		c->setAngularUpperLimit(btVector3(0, 0, 0));
	    }
	    else
	    {
		c->setLinearLowerLimit(btVector3(0, 0, 0));
		c->setLinearUpperLimit(btVector3(0, 0, 0));
		c->setAngularLowerLimit(btVector3(1, 1, 1));
		c->setAngularUpperLimit(btVector3(-1, -1, -1));
	    }
	}
	else if(!rotational)
	{
	    const SIM_ConRelConeTwist *cone_twist =
				SIM_DATA_CASTCONST(rel, SIM_ConRelConeTwist);
	    if(cone_twist)
	    {
		// get the rotation, the twist axis is the x axis by default,
		// to change that, we find angle between the set axis and the
		// axis axis, and rotate the anchor transformation by that
		// amount.
		btVector3 twist_axis(simbtVector3(cone_twist->getTwistAxis()));
		btVector3 x_axis(1, 0, 0);

		// only set the frame rotation if the axis is not the x-axis
		if(twist_axis != x_axis)
		{
		    btQuaternion rot(x_axis.cross(twist_axis),
				     x_axis.angle(twist_axis));

		    localA.setRotation(rot);
		    localB.setRotation(rot);
		}

		// cone twist
		btConeTwistConstraint *c = new btConeTwistConstraint(*bodyA, *bodyB, localA, localB);
		myConstraint = c;

		// get the limits
		c->setLimit(SYSdegToRad(cone_twist->getSwingSpan1()),
			    SYSdegToRad(cone_twist->getSwingSpan2()),
			    SYSdegToRad(cone_twist->getTwistSpan()),
			    cone_twist->getSoftness(),
			    cone_twist->getBiasFactor(),
			    cone_twist->getRelaxationFactor());
	    }
	}

	// apply constraint
	if(myConstraint)
	{
	    myDynamicsWorld->addConstraint(myConstraint, false);
	    myConstraint->enableFeedback(true);
	    rel->setStateErrorCode(SIM_CONRELERR_NONE);
	}
    }

    ~simBulletConstraint()
    {
	if(myConstraint)
	{
	    myDynamicsWorld->removeConstraint(myConstraint);
	    delete myConstraint;
	}
	if(myBody)
	{
	    myDynamicsWorld->removeRigidBody(myBody);
	    delete myBody->getCollisionShape();
	    delete myBody;
	}
    }

    // informs constraints about the forces applied to satisfy them
    void updateConstraintState(const SIM_Time &timestep)
    {
	if(myConstraint)
	{
	    myRel->setStateForce(myConstraint->getAppliedImpulse() / timestep);
	    // FIXME: set the distance
	    //myRel->setStateDistance();
	}
    }

private:
    // constraint being represented in Bullet
    SIM_ConRel *myRel;

    // the Bullet simulation
    btDiscreteDynamicsWorld *myDynamicsWorld;

    // the Bullet constraint
    btTypedConstraint *myConstraint;

    // body added to describe constraints to worldspace positions
    btRigidBody *myBody;
};

// helper class to discover glue connectivity
class simGlueHelper
{
    // describes collection of objects that are connected by glue
    struct Chunk
    {
	Chunk(int index) : myIndex(index) {}

	// index of this chunk in simGlueHelper::myChunks
	int myIndex;

	// index of member objects
	UT_IntArray myMembers;
    };

public:
    simGlueHelper(int n)
    {
	// initially none of the objects belong to chunks
	for(int i = 0; i < n; ++i)
	    myObjects.append(0);
    }

    ~simGlueHelper()
    {
	while(myChunks.entries())
	    delete myChunks.removeLast();
    }

    // maps from an object_id to the chunk_id to which it is glued, or -1
    int getChunkFromObject(int object_id) const
    {
	Chunk *chunk = myObjects(object_id);
	return chunk ? chunk->myIndex : -1;
    }

    // returns a list of object ids that are pieces of the chunk
    const UT_IntArray &getChunkPieces(int chunk_id) const { return myChunks(chunk_id)->myMembers; }

    void glueObjects(int object_a, int object_b)
    {
	// no need to glue an object to itself
	if(object_a == object_b)
	    return;

	Chunk *chunk_a = myObjects(object_a);
	Chunk *chunk_b = myObjects(object_b);

	// check if they are already glued together
	if(chunk_a && chunk_a == chunk_b)
	    return;

	// swap if necessary to ensure we update the minimum number of items
	if(!chunk_a || (chunk_b && chunk_a->myMembers.entries() < chunk_b->myMembers.entries()))
	{
	    // swap 'object_a' and 'object_b' so chunk_b will have fewer
	    // members
	    int temp = object_a;
	    object_a = object_b;
	    object_b = temp;

	    chunk_a = myObjects(object_a);
	    chunk_b = myObjects(object_b);
	}

	// create chunk_a if it didn't exist yet
	if(!chunk_a)
	{
	    chunk_a = new Chunk(myChunks.entries());
	    myChunks.append(chunk_a);
	    chunk_a->myMembers.append(object_a);
	    myObjects(object_a) = chunk_a;
	}

	if(chunk_b)
	{
	    // merge chunk_b into chunk_a

	    // transfer all of chunk_b's members into chunk_a
	    for(int i = chunk_b->myMembers.entries() - 1; i >= 0; --i)
	    {
		int object_id = chunk_b->myMembers(i);
		chunk_a->myMembers.append(object_id);
		myObjects(object_id) = chunk_a;
	    }
	    // remove chunk_b and shuffle remaining chunks to fill newly
	    // created hole
	    Chunk *temp = myChunks.removeLast();
	    if(temp != chunk_b)
	    {
		temp->myIndex = chunk_b->myIndex;
		myChunks(chunk_b->myIndex) = temp;
	    }
	    delete chunk_b;
	}
	else
	{
	    // merge object_b into chunk_a
	    chunk_a->myMembers.append(object_b);
	    myObjects(object_b) = chunk_a;
	}
    }

private:
    UT_RefArray<Chunk *> myObjects;
    UT_PtrArray<Chunk *> myChunks;
};

class simBulletState
{
public:
    simBulletState(SIM_Engine &engine) : myRefCount(1), myDynamicsWorld(0), myEngine(engine), myTime(0)
    {
	myCollisionConfiguration = new btDefaultCollisionConfiguration();
	myCollisionConfiguration->setConvexConvexMultipointIterations();
	myDispatcher = new btCollisionDispatcher(myCollisionConfiguration);
	myBroadphase = new btDbvtBroadphase();
	mySolver = new btSequentialImpulseConstraintSolver();
	reset();
    }

    ~simBulletState()
    {
	// these should not presist between solves
	UT_ASSERT(myDopIdLookup.empty());
	UT_ASSERT(!myGlueNetworks.entries());
	UT_ASSERT(!myConstraints.entries());
	removeObjects();
	delete myDynamicsWorld;
	delete mySolver;
	delete myBroadphase;
	delete myDispatcher;
	delete myCollisionConfiguration;
	cout << "deleting the bullet state" << endl;
    }

    // add/remove references to the simulation state
    void ref() { ++myRefCount; }
    void unref() { if(--myRefCount == 0) delete this; }

    // simulation time of the Bullet simulation
    const SIM_Time &getSimulationTime() const { return myTime; }

    // resets entire Bullet simulation
    void reset()
    {
	removeObjects();
	delete myDynamicsWorld;
	myDynamicsWorld = new btDiscreteDynamicsWorld(myDispatcher, myBroadphase, mySolver, myCollisionConfiguration);
	myDynamicsWorld->setGravity(btVector3(0, 0, 0));
	myDynamicsWorld->clearForces();
	myDynamicsWorld->setInternalTickCallback(tickCallback, this);
    }

    // set solver parameters
    void setSolverParameters(int num_iteration, bool split_impulse, btScalar threshold)
    {
	btContactSolverInfo &info = myDynamicsWorld->getSolverInfo();
	info.m_numIterations = num_iteration;
	info.m_splitImpulse = split_impulse;
	info.m_splitImpulsePenetrationThreshold = threshold;
    }

    // export simulation data to Bullet
    void updateFromObjects(SIM_SolverBulletHolladay *solver, SIM_ObjectArray &objects)
    {
	UT_HashTable known_dopids;

	// flag all existing objects as dead
	int n = myObjects.entries();
	for(int i = 0; i < n; ++i)
	{
	    simBulletObject *obj = myObjects(i);
	    obj->setUnused();

	    UT_Hash_Int64 hashkey(obj->getDopId());
	    UT_Thing thing((long)i);
	    known_dopids.addSymbol(hashkey, thing);
	}

	// add / update objects
	n = objects.entries();
	for(int i = 0; i < n; ++i)
	{
	    SIM_Object *obj = objects(i);

	    UT_Hash_Int64 hashkey(obj->getObjectId());
	    UT_Thing thing;
	    if(!known_dopids.findSymbol(hashkey, &thing))
	    {
		// create new Bullet object
		int idx = myObjects.entries();
		thing = (long)idx;
		
		myObjects.append(new simBulletObject(idx, myDynamicsWorld));
		known_dopids.addSymbol(hashkey, thing);
	    }
	    // update Bullet object
	    myObjects((long)thing)->updateFromObject(solver, *obj, false);
	}

	// add / update affectors
	UT_HashTable known_colliders;
	for(int i = 0; i < n; ++i)
	{
	    SIM_Object *obj = objects(i);

	    UT_TokenString *ts = UT_TokenString::allocString();
	    obj->getColliderInfoTokenString(*ts);
	    UT_Hash_TokenString hash(ts);
	    UT_TokenString::freeString(ts);
	    UT_Thing dummy;

	    if(!known_colliders.findSymbol(hash, &dummy))
	    {
		known_colliders.addSymbol(hash, dummy);

		SIM_ColliderInfoArray colliders;
		obj->getColliderInfo(colliders);
		int ncolliders = colliders.entries();
		for(int j = 0; j < ncolliders; ++j)
		{
		    SIM_Object *affector = colliders(j).getAffector();

		    UT_Hash_Int64 hashkey(affector->getObjectId());
		    UT_Thing thing;
		    if(!known_dopids.findSymbol(hashkey, &thing))
		    {
			// create new Bullet object
			int idx = myObjects.entries();
			thing = (long)idx;
			
			myObjects.append(new simBulletObject(idx, myDynamicsWorld));
			known_dopids.addSymbol(hashkey, thing);
		    }
		    // update Bullet object
		    myObjects((long)thing)->updateFromObject(solver, *affector, true);
		}
	    }
	}

	// removed dead objects and finish setup of living objects
	for(int i = myObjects.entries() - 1; i >= 0; --i)
	{
	    simBulletObject *obj = myObjects(i);
	    if(obj->isUnused())
	    {
		// remove object from its glue chunk
		unglue(obj);

		simBulletObject *temp = myObjects.removeLast();
		if(obj != temp)
		{
		    temp->setIndex(i);
		    myObjects(i) = temp;
		}
		delete obj;
	    }
	    else
		obj->setCollisionFlags();
	}
    }

    // update myGluedChunks
    void glueObjects(SIM_ObjectArray &objects)
    {
	buildGlueNetworks(objects);

	// figure out which items should be glued together
	simGlueHelper helper(myObjects.entries());
	for(int i = myGlueNetworks.entries() - 1; i >= 0; --i)
	{
	    simGlueNetwork *network = myGlueNetworks(i);
	    for(int j = network->getNumEdges() - 1; j >= 0; --j)
		helper.glueObjects(network->getIndex0(j), network->getIndex1(j));
	}
 
	// update glued chunks
	UT_HashTable known_chunks;
	for(int i = myObjects.entries() - 1; i >= 0; --i)
	{
	    int chunk_id = helper.getChunkFromObject(i);
	    if(chunk_id < 0)
	    {
		// this object should not be glued
		unglue(myObjects(i));
		continue;
	    }

	    UT_Hash_Int64 hashkey(chunk_id);
	    UT_Thing dummy;
	    if(known_chunks.findSymbol(hashkey, &dummy))
	    {
		// all pieces of this chunk have been processed
		continue;
	    }

	    simBulletObject *obj = myObjects(i);
	    simBulletGluedChunk *chunk = obj->getGluedChunk();
	    if(!chunk)
	    {
		// allocate a new chunk
		chunk = new simBulletGluedChunk(myGluedChunks.entries(),
						myDynamicsWorld);
		myGluedChunks.append(chunk);
	    }
	    else
	    {
		// remove all pieces that don't belong in this chunk
		for(int j = chunk->getNumPieces() - 1; j >= 0; --j)
		{
		    simBulletObject *piece = chunk->getPiece(j);
		    if(helper.getChunkFromObject(piece->getIndex()) != chunk_id)
			unglue(piece);
		}
	    }

	    // add all pieces that belong to this chunk
	    const UT_IntArray &pieces = helper.getChunkPieces(chunk_id);
	    for(int j = pieces.entries() - 1; j >= 0; --j)
	    {
		simBulletObject *piece = myObjects(pieces(j));
		simBulletGluedChunk *old_chunk = piece->getGluedChunk();
		if(old_chunk != chunk)
		{
		    if(old_chunk)
			unglue(piece);
		    chunk->addPiece(piece);
		}
	    }
	    chunk->updateBody(myEngine);
	    known_chunks.addSymbol(hashkey, dummy);
	}
    }

    // apply forces
    void applyForces(SIM_ObjectArray &objects, const SIM_Time &timestep)
    {
	// clear all forces on glue objects
	for(int i = myGluedChunks.entries() - 1; i >= 0; --i)
	{
	    simBulletGluedChunk *chunk = myGluedChunks(i);
	    btRigidBody *body = chunk->getBody();
	    if(body)
		body->clearForces();
	}
	for(int i = myObjects.entries() - 1; i >= 0; --i)
	{
	    simBulletObject *obj = myObjects(i);
	    // clear all forces on the object
	    btRigidBody *body = obj->getBody();
	    if(body)
		body->clearForces();

	    // get correct body to recieve forces
	    simBulletGluedChunk *chunk = obj->getGluedChunk();
	    if(chunk)
		body = chunk->getBody();

	    // don't apply forces to affectors
	    if(!body || body->getCollisionFlags() == btCollisionObject::CF_KINEMATIC_OBJECT)
		continue;

	    SIM_Object *simobj = objects.findObjectById(obj->getDopId());
	    if(!simobj)
		continue;

	    const RBD_State *rbdstate = SIM_DATA_GETCONST(*simobj, "Position", RBD_State);
	    if(!rbdstate)
		continue;

	    // apply impact and feedback forces
	    for(int s = 0; s < 2; ++s)
	    {
		static const char *dataname[] = 
		{
		    SIM_IMPACTS_DATANAME,
		    SIM_FEEDBACK_DATANAME
		};

		const SIM_Impacts *impacts =
			SIM_DATA_GETCONST(*simobj, dataname[s], SIM_Impacts);
		if(impacts)
		{
		    for(int j = 0; j < impacts->getNumImpacts(); ++j)
		    {
			btVector3 bt_force(simbtVector3((impacts->getImpulse(j)/timestep) * impacts->getNormal(j)));
			// force is applied at an offset from center_of_mass
			btVector3 bt_torque = (simbtVector3(impacts->getPosition(j)) - body->getCenterOfMassPosition()).cross(bt_force);

			body->applyCentralForce(bt_force);
			body->applyTorque(bt_torque);
		    }
		}
	    }

	    // apply forces
	    SIM_ConstDataArray forces;
	    simobj->filterConstSubData(forces, 0,
				       SIM_DataFilterByType("SIM_Force"),
				       SIM_FORCES_DATANAME,
				       SIM_DataFilterNone());

	    btVector3 bt_pos = simbtTransform(rbdstate->getPosition(), rbdstate->getOrientation(), rbdstate->getPivot()) * obj->getCenterOfMass().getOrigin();
	    UT_Vector3 pos(utVector3(bt_pos));
	    btVector3 gravity(0, 0, 0);
	    for(int i = 0; i < forces.entries(); ++i)
	    {
		const SIM_ForceGravity *gravity_force = SIM_DATA_CASTCONST(forces(i), SIM_ForceGravity);
		if(gravity_force)
		{
		    gravity += simbtVector3(gravity_force->getGravity());
		    continue;
		}

		const SIM_Force *f = SIM_DATA_CASTCONST(forces(i), SIM_Force);

		UT_Vector3 force, torque;
		f->getForce(*simobj, pos,
			    rbdstate->getVelocityAtPosition(pos),
			    rbdstate->getAngularVelocityDegrees(),
			    rbdstate->getMass(), force, torque);

		btVector3 bt_force(simbtVector3(force));
		btVector3 bt_torque(simbtVector3(torque));
		if(chunk)
		{
		    // force is applied at an offset from center_of_mass
		    bt_torque += (bt_pos - body->getCenterOfMassPosition()).cross(bt_force);
		}

		body->applyCentralForce(bt_force);
		body->applyTorque(bt_torque);
	    }
	    body->setGravity(gravity);
	}
    }
	
    // add constraints to Bullet simulation
    void applyConstraints(SIM_ObjectArray &objects)
    {
	// these should not presist between solves
	UT_ASSERT(!myConstraints.entries());

	const SIM_Time &t = myEngine.getSimulationTime();
	for(int i = myObjects.entries() - 1; i >= 0; --i)
	{
	    simBulletObject *obj = myObjects(i);
	    SIM_Object *simobj = objects.findObjectById(obj->getDopId());
	    if(!simobj)
		continue;

	    // spatial constraints
	    SIM_DataFilterByType pObjFilter("SIM_ConAnchorObjSpatial");
	    SIM_DataFilterByType pFilter("SIM_ConAnchorSpatial");
	    SIM_ConstraintIterator it(*simobj, 0, &pObjFilter, &pFilter, t);
	    for(; !it.atEnd(); it.advance())
	    {
		SIM_ConRel *conrel = it.getConRel();
		const SIM_ConAnchorObjSpacePos *anchor0 = static_cast<const SIM_ConAnchorObjSpacePos *>(it.getCurrentAnchor());
		const SIM_ConAnchorSpatial *anchor1 = static_cast<const SIM_ConAnchorSpatial *>(it.getGoalAnchor());

		simBulletObject *goalobj = 0;
		const SIM_Object *goal = anchor1->getReferencedObject(t);
		if(goal)
		{
		    UT_Hash_Int64 hashkey(goal->getObjectId());
		    UT_Thing thing;
		    if(myDopIdLookup.findSymbol(hashkey, &thing))
			goalobj = myObjects((long)thing);
		}

		// world space position of anchors
		btTransform transA(btQuaternion(0, 0, 0, 1), simbtVector3(anchor0->getPosition(t)));
		btTransform transB(btQuaternion(0, 0, 0, 1), simbtVector3(anchor1->getPosition(t)));

		myConstraints.append(new simBulletConstraint(conrel, myDynamicsWorld, obj, transA, goalobj, transB, simbtVector3(anchor1->getVelocity(t)), false));
	    }

	    // rotational constraints
	    SIM_DataFilterByType rObjFilter("SIM_ConAnchorObjRotational");
	    SIM_DataFilterByType rFilter("SIM_ConAnchorRotational");

	    SIM_ConstraintIterator it2(*simobj, 0, &rObjFilter, &rFilter, t);
	    for(; !it2.atEnd(); it2.advance())
	    {
		SIM_ConRel *conrel = it2.getConRel();
		const SIM_ConAnchorObjSpaceRot *anchor0 = static_cast<const SIM_ConAnchorObjSpaceRot *>(it2.getCurrentAnchor());
		const SIM_ConAnchorRotational *anchor1 = static_cast<const SIM_ConAnchorRotational *>(it2.getGoalAnchor());

		simBulletObject *goalobj = 0;
		const SIM_Object *goal = anchor1->getReferencedObject(t);
		if(goal)
		{
		    UT_Hash_Int64 hashkey(goal->getObjectId());
		    UT_Thing thing;
		    if(myDopIdLookup.findSymbol(hashkey, &thing))
			goalobj = myObjects((long)thing);
		}

		// world space position of anchors
		btTransform transA(simbtQuaternion(anchor0->getOrientation(t)),
				   simbtVector3(anchor0->getGuidePosition(t)));
		btTransform transB(simbtQuaternion(anchor1->getOrientation(t)),
				   simbtVector3(anchor1->getGuidePosition(t)));

		myConstraints.append(new simBulletConstraint(conrel, myDynamicsWorld, obj, transA, goalobj, transB, simbtVector3(anchor1->getAngularVelocity(t)), true));
	    }
	}
    }
	
	// EDIT // These functions saves out and restores the transforms of all the glue pieces
	void saveTransforms( SIM_ObjectArray &objects )
	{
		for(int i = myObjects.entries() - 1; i >= 0; --i)
		{
			simBulletObject *obj = myObjects(i);
			SIM_Object *simobj = objects.findObjectById(obj->getDopId());
			if(!simobj)
				continue;
			
			btRigidBody *body = obj->getBody();
			if(!body)
				continue;
			
			simBulletGluedChunk *chunk = obj->getGluedChunk();
			if ( chunk )
			{
				chunk->saveTransforms();
			}  // if
			else
			{
				obj->saveTransforms();
			}  // else
		}  // for i
	}  // saveGlueObjectTransforms()
	
	void resetTransforms( SIM_ObjectArray &objects )
	{
		for(int i = myObjects.entries() - 1; i >= 0; --i)
		{
			simBulletObject *obj = myObjects(i);
			SIM_Object *simobj = objects.findObjectById(obj->getDopId());
			if(!simobj)
				continue;
			
			btRigidBody *body = obj->getBody();
			if(!body)
				continue;
			
			simBulletGluedChunk *chunk = obj->getGluedChunk();
			if ( chunk )
			{
				chunk->resetTransforms();
			}  // if
			else
			{
				obj->resetTransforms();
			}  // else
		}  // for i
	}  // resetTransforms()
	//////////
	
	
	
    // advance the Bullet simulation
    void stepSimulation(const SIM_Time &t, const SIM_Time &timestep,
			int substeps, const SIM_ObjectArray &feedbacktoobjects)
    {
	SIM_Time step;
	if(substeps == 1)
	    step = timestep;
	else
	{
	    step = timestep / substeps;
	    ++substeps; // add 1 for roundoff errors
	}

	// clear impulses
	for(int i = myObjects.entries() - 1; i >= 0; --i)
	    myObjects(i)->setImpulses(0);

	// step simulation
	myFeedbackToObjects = &feedbacktoobjects;
	myDynamicsWorld->stepSimulation(timestep, substeps, step);
	myTime = t;
    }

    // remove constraints from Bullet simulation and make state transitions
    void removeConstraints(SIM_ObjectArray &objects, const SIM_Time &timestep)
    {
	while(myConstraints.entries())
	{
	    simBulletConstraint *c = myConstraints.removeLast();
	    c->updateConstraintState(timestep);
	    delete c;
	}

	const SIM_Time &t = myEngine.getSimulationTime();
	for(int i = objects.entries() - 1; i >= 0; --i)
	    SIM_ConstraintIterator::makeStateTransitions(*objects(i), t);
    }

    // import simulation results from Bullet
    void updateObjects(SIM_ObjectArray &objects, const SIM_Time &timestep)
    {
	for(int i = myObjects.entries() - 1; i >= 0; --i)
	{
	    simBulletObject *obj = myObjects(i);
	    SIM_Object *simobj = objects.findObjectById(obj->getDopId());
	    if(!simobj)
		continue;

	    btRigidBody *body = obj->getBody();
	    if(!body)
		continue;

	    simBulletGluedChunk *chunk = obj->getGluedChunk();
	    if(chunk)
	    {
		// update object's transform from the chunk
		btRigidBody *chunk_body = chunk->getBody();
		body->setCenterOfMassTransform(
				chunk_body->getCenterOfMassTransform() *
				chunk->getCenterOfMass().inverse() *
				body->getCenterOfMassTransform());

		// update object's velocity from the chunk
		const btVector3 &avel = chunk_body->getAngularVelocity();
		body->setLinearVelocity(
			chunk_body->getLinearVelocity() +
			avel.cross(body->getCenterOfMassPosition() -
				   chunk_body->getCenterOfMassPosition()));
		body->setAngularVelocity(avel);
	    }

	    RBD_State *rbdstate = SIM_DATA_GET(*simobj, "Position", RBD_State);
	    if(!rbdstate)
		continue;

	    rbdstate->setInheritVelocity(false);
	    rbdstate->decayGlueImpulse(timestep);

	    // update position
	    const btTransform &center_of_mass = obj->getCenterOfMass();
	    const btTransform &xform = body->getCenterOfMassTransform() * center_of_mass.inverse();
	    const btVector3 &pivot = center_of_mass.getOrigin();
	    rbdstate->setPivot(utVector3(pivot));
	    rbdstate->setOrientation(utQuaternion(xform.getRotation()));
	    rbdstate->setPosition(utVector3(xform * pivot - pivot));

	    // update velocity
	    rbdstate->setVelocity(utVector3(body->getLinearVelocity()));
	    rbdstate->setAngularVelocity(utVector3(body->getAngularVelocity()));
	}
    }

    // weaken or break glue
    void updateGlue(SIM_ObjectArray &objects, const SIM_Time &timestep)
    {
	myDopIdLookup.clear();
	
	// remove networks as we process them
	while(myGlueNetworks.entries())
	{
	    simGlueNetwork *network = myGlueNetworks.removeLast();
	    SIM_GlueNetworkRelationship *rel = network->getRelationship();
	    if(rel)
	    {
		SIM_GeometryCopy *geo = SIM_DATA_CREATE(*rel, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
		if(geo)
		{
		    GU_DetailHandleAutoWriteLock gdl(geo->lockGeometry());
		    GU_Detail *gdp = gdl.getGdp();

		    fpreal rate = rel->getPropagateRate();
		    fpreal halflife = rel->getHalfLife();
		    fpreal strength = rel->getStrength();

		    UT_String rate_attrib, halflife_attrib, strength_attrib;
		    rel->getPropagateRateAttrib(rate_attrib);
		    rel->getHalfLifeAttrib(halflife_attrib);
		    rel->getStrengthAttrib(strength_attrib);
		    GA_RWAttributeRef rate_ref = gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, rate_attrib, 1);
		    GA_RWAttributeRef halflife_ref = gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, halflife_attrib, 1);
		    GA_RWAttributeRef strength_ref = gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, strength_attrib, 1);
		    GA_RWAttributeRef impact_ref = gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, "impact", 1);
		    if(!impact_ref.isValid())
			impact_ref = gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "impact", 1);

		    // glue network
		    UT_FloatArray impulses;
		    UT_FloatArray weight;
		    UT_FloatArray accum;
		    const GA_IndexMap &map = gdp->getPointMap();
		    int n = myObjects.entries();
		    for(int i = 0; i < n; ++i)
		    {
			// initialize impulse data for diffusion pass
			simBulletObject *obj = myObjects(i);
			fpreal impulse = obj->getImpulses();
			impulses.append(impulse);
			weight.append(0);
			accum.append(0);
		    }

		    // diffuse values
		    for(int p = rel->getPropagateIterations() - 1; p >= 0; --p)
		    {
			// objects diffuse to themselves with a rate of 1
			for(int i = 0; i < n; ++i)
			{
			    weight(i) = 1;
			    accum(i) = 0;
			}

			// find rate impulses diffuse away from each object
			for(int e = network->getNumEdges() - 1; e >= 0; --e)
			{
			    fpreal w = rate;
			    if(rate_ref.isValid())
			    {
				GA_Index pr = network->getPrimitiveNumber(e);
				w *= gdp->primitives()(pr)->getValue<fpreal>(rate_ref);
			    }
			    w = SYSclamp(w, 0.0, 1.0);
			    weight(network->getIndex0(e)) += w;
			    weight(network->getIndex1(e)) += w;
			}

			// transfer impulses across glue constraint
			for(int e = network->getNumEdges() - 1; e >= 0; --e)
			{
			    fpreal w = rate;
			    if(rate_ref.isValid())
			    {
				GA_Index pr = network->getPrimitiveNumber(e);
				w *= gdp->primitives()(pr)->getValue<fpreal>(rate_ref);
			    }
			    w = SYSclamp(w, 0.0, 1.0);
			    int idx0 = network->getIndex0(e);
			    int idx1 = network->getIndex1(e);
			    accum(idx0) += w * impulses(idx1) / weight(idx1);
			    accum(idx1) += w * impulses(idx0) / weight(idx0);
			}

			for(int i = 0; i < n; ++i)
			{
			    impulses(i) /= weight(i);
			    impulses(i) += accum(i);
			}
		    }

		    // update impact value and determine which primitives to
		    // remove
		    GA_PrimitiveGroup *grp = gdp->newInternalPrimitiveGroup();
		    for(int e = network->getNumEdges() - 1; e >= 0; --e)
		    {
			int idx0 = network->getIndex0(e);
			int idx1 = network->getIndex1(e);

			// update glue network guide geometry
			GA_Offset pt0 = map.offsetFromIndex(network->getPoint0(e));
			btRigidBody *body0 = myObjects(idx0)->getBody();
			if(body0)
			    gdp->setPos3(pt0, utVector3(body0->getCenterOfMassPosition()));
			GA_Offset pt1 = map.offsetFromIndex(network->getPoint1(e));
			btRigidBody *body1 = myObjects(idx1)->getBody();
			if(body1)
			    gdp->setPos3(pt1, utVector3(body1->getCenterOfMassPosition()));

			GA_Index pr = network->getPrimitiveNumber(e);
			GEO_Primitive *prim = gdp->primitives()(pr);
			fpreal impact = 0;

			// compare total impacts against strength threshold
			fpreal threshold = strength;
			// This is a bit complicated as we want to handle
			// infinite strengths which are negative.
			if (strength_ref.isValid())
			{
			    if (threshold < 0)
			    {
				// Ignore the threshold and copy the
				// strength attribute.  Our default is
				// a -1, so this lets people quickly
				// override.
				threshold = prim->getValue<fpreal>(strength_ref);
			    }
			    else
			    {
				fpreal	primstr = prim->getValue<fpreal>(strength_ref);
				if (primstr < 0)
				{
				    // Directly override the user threshold
				    // as we want this to be strong
				    // no matter what the scale (Even 0!!)
				    threshold = primstr;
				}
				else
				{
				    // Combine the two
				    threshold *= primstr;
				}
			    }
			}
			if(threshold >= 0)
			{
			    if(impact_ref.isValid())
			    {
				impact = prim->getValue<fpreal>(impact_ref);
				if(impact)
				{
				    // decay glue impacts
				    fpreal w = halflife;
				    if(halflife_ref.isValid())
					w *= prim->getValue<fpreal>(halflife_ref);

				    if(SYSisLessOrEqual(w, 0))
					impact = 0;
				    else
					impact *= SYSpow((fpreal)0.5, (fpreal)timestep / w);
				}
			    }

			    // add diffused impulses
			    impact += impulses(idx0) + impulses(idx1);

			    if(impact > threshold)
			    {
				// break the glue
				grp->addIndex(pr);
				impact = 0;
			    }
			}

			// record total impact data
			if(impact_ref.isValid())
			    prim->setValue(impact_ref, impact);
		    }

		    gdp->deletePrimitives(*grp);
		    gdp->destroyPrimitiveGroup(grp);
		}
	    }
	    else
	    {
		// RBD glue
		for(int e = network->getNumEdges() - 1; e >= 0; --e)
		{
		    simBulletObject *obj = myObjects(network->getIndex0(e));
		    SIM_Object *simobj = objects.findObjectById(obj->getDopId());
		    if(!simobj)
			continue;

		    RBD_State *rbdstate = SIM_DATA_GET(*simobj, "Position", RBD_State);
		    if(!rbdstate)
			continue;

		    fpreal threshold = rbdstate->getGlueThreshold();
		    if(threshold < 0)
			continue;
			
		    fpreal impulses = obj->getImpulses();
		    if(rbdstate->getGlueImpulse() + impulses > threshold)
		    {
			// break glue
			rbdstate->setGlueObject("");
			rbdstate->setGlueImpulse(0);
			
		    }
		    else
			rbdstate->accumulateGlueImpulse(impulses);
		}
	    }
	    delete network;
	}
    }
	
	// EDIT // Functions for detecting collisions (performDiscreteCollisionDetection()) and ungluing any objects that were contacted (removeContactedGlueObjects())
	void removeContactedGlueObjects( SIM_ObjectArray &objects )
	{
		int numObjects = myObjects.entries();
		for ( int i = 0; i < numObjects; i++ )
		{
			simBulletObject *obj = myObjects(i);
			if ( obj->getCollided())
			{
				simBulletGluedChunk *chunk = obj->getGluedChunk();
				if ( chunk )
				{
					SIM_Object *simobj = objects.findObjectById(obj->getDopId());
					if(!simobj)
						continue;
					
					RBD_State *rbdstate = SIM_DATA_GET(*simobj, "Position", RBD_State);
					if(!rbdstate)
						continue;
					
					rbdstate->setGlueObject("");
					rbdstate->setGlueImpulse(0);			
				}  // if
				obj->setCollided( false );
			}  // if
		}  // for i
	}  // removeContactedGlueObjects()
	
	void performDiscreteCollisionDetection( SIM_ObjectArray &objects, const SIM_Time &timestep )
	{
		btDispatcherInfo& dispatchInfo = myDynamicsWorld->getDispatchInfo();
		SIM_Time t = timestep;
		dispatchInfo.m_timeStep = t;
		dispatchInfo.m_stepCount = 0;
		dispatchInfo.m_debugDraw = myDynamicsWorld->getDebugDrawer();
		
		btCollisionDispatcher *dispatcher = (btCollisionDispatcher*)myDynamicsWorld->getDispatcher();
		int numManifolds = dispatcher->getNumManifolds();
		
		int numChunks = myGluedChunks.entries();
		for ( int i = 0; i < numChunks; i++ )
		{
			simBulletGluedChunk* curChunk = myGluedChunks(i);
			btRigidBody* body = curChunk->getBody();
			if ( !body )
				continue;
			
			body->applyGravity();
			
			body->integrateVelocities( t );
			
			body->applyDamping( t );
			
			body->predictIntegratedTransform(t,body->getInterpolationWorldTransform());
		}  // for i
		
		int numBodies = myObjects.entries();
		for ( int i = 0; i < numBodies; i++ )
		{
			simBulletObject *obj = myObjects(i);
			simBulletGluedChunk *chunk = obj->getGluedChunk();
			if ( chunk )
				continue;
			
			btRigidBody* body = obj->getBody();
			if ( !body )
				continue;
			
			body->applyGravity();
			
			body->integrateVelocities( t );
			
			body->applyDamping( t );
			
			body->predictIntegratedTransform(t,body->getInterpolationWorldTransform());
		}  // for i
		
		for ( int i = 0; i < numChunks; i++ )
		{
			simBulletGluedChunk* curChunk = myGluedChunks(i);
			btRigidBody* body = curChunk->getBody();
			if ( !body )
				continue;
			
			btTransform predictedTrans;
			body->predictIntegratedTransform( t, predictedTrans );
			body->proceedToTransform( predictedTrans );
		}  // for i
		
		for ( int i = 0; i < numBodies; i++ )
		{
			simBulletObject *obj = myObjects(i);
			simBulletGluedChunk *chunk = obj->getGluedChunk();
			if ( chunk )
				continue;
			
			btRigidBody* body = obj->getBody();
			if ( !body )
				continue;
			
			btTransform predictedTrans;
			body->predictIntegratedTransform( t, predictedTrans );
			body->proceedToTransform( predictedTrans );
		}  // for i
		
		myDynamicsWorld->performDiscreteCollisionDetection();
		
		myDynamicsWorld->getSolverInfo().m_timeStep = t;
		
		dispatcher = (btCollisionDispatcher*)myDynamicsWorld->getDispatcher();
		numManifolds = dispatcher->getNumManifolds();
		for ( int i = 0; i < numManifolds; i++ )
		{
			btPersistentManifold* contactManifold = myDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			for ( int j = 0; j < contactManifold->getNumContacts(); j++ )
			{
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if (pt.getDistance() <= contactManifold->getContactProcessingThreshold())
				{
					btRigidBody* body0 = (btRigidBody*)contactManifold->getBody0();
					btRigidBody* body1 = (btRigidBody*)contactManifold->getBody1();
						
					simBulletObject *objA = simObjectFromShapeIdx( pt.m_index0, body0 );
					simBulletObject *objB = simObjectFromShapeIdx( pt.m_index1, body1 );
					
					if ( objA )
					{
						if ( objA->getGluedChunk() == objB->getGluedChunk() )
							continue;
							
						btCollisionShape *shape = body0->getCollisionShape();
						btCompoundShape *comp = (btCompoundShape *)(shape);
							
						// Compute the impulse against objA
						btScalar depth = 0.f;
						btScalar appliedImpulse = resolveCollision( body0, body1, pt.m_positionWorldOnA, pt.m_normalWorldOnB, myDynamicsWorld->getSolverInfo(), depth );	// resolveSingleCollision included from btContactConstraint.cpp
							
						// Get the impulse threshold
						SIM_Object *simobj = objects.findObjectById(objA->getDopId());
						if(!simobj)
							continue;
						
						RBD_State *rbdstate = SIM_DATA_GET(*simobj, "Position", RBD_State);
						if(!rbdstate)
							continue;
						
						fpreal threshold = rbdstate->getGlueThreshold();
						
						// If the impulse is high enough, label this object to break
						if ( threshold >= 0 && appliedImpulse > threshold )
							objA->setCollided( true );		// Mark the collided objects to later determine which glued pieces need to break off
					}  // if
					if ( objB )
					{
						if ( objA->getGluedChunk() == objB->getGluedChunk() )
							continue;
							
						// Compute the impulse against objB
						btScalar depth = 0.f;
						btScalar appliedImpulse = resolveCollision( body1, body0, pt.m_positionWorldOnB, pt.m_normalWorldOnB, myDynamicsWorld->getSolverInfo(), depth );	// resolveSingleCollision included from btContactConstraint.cpp
							
						// Get the impulse threshold
						SIM_Object *simobj = objects.findObjectById(objB->getDopId());
						if(!simobj)
							continue;
						
						RBD_State *rbdstate = SIM_DATA_GET(*simobj, "Position", RBD_State);
						if(!rbdstate)
							continue;
						
						fpreal threshold = rbdstate->getGlueThreshold();
						
						// If the impulse is high enough, label this object to break
						if ( threshold >= 0 && appliedImpulse > threshold )
							objB->setCollided( true );		// Mark the collided objects to later determine which glued pieces need to break off
					}  // if
				}  // if
			}  // for j
		}  // for i
	}  // performDiscreteCollisionDetection()
	//////////

private:
    // remove all objects in the Bullet simulation
    void removeObjects()
    {
	while(myGluedChunks.entries())
	    delete myGluedChunks.removeLast();
	while(myObjects.entries())
	    delete myObjects.removeLast();
    }

    // utility method to find the index of the named object in myObjects
    int indexFromObjectName(const UT_HashTable &objectnames, const char *name)
    {
	if(!name || !*name)
	    return -1;

	// try parsing name as a DOP ID
	const char *tmp = name;
	int dopid = 0;
	while(*tmp >= '0' && *tmp <= '9')
	{
	    dopid = 10 * dopid + *tmp - '0';
	    ++tmp;
	}

	if(*tmp)
	{
	    // failed to parse as a DOP ID, now look for object by name
	    UT_Hash_String hashkey(name);
	    UT_Thing thing;
	    if(objectnames.findSymbol(hashkey, &thing))
		return (long)thing;
	}
	else
	{
	    // we successfully parsed name as a DOP ID
	    UT_Hash_Int64 hashkey(dopid);
	    UT_Thing thing;
	    if(myDopIdLookup.findSymbol(hashkey, &thing))
		return (long)thing;
	}
	return -1;
    }

    // rebuilds myGlueNetworks
    void buildGlueNetworks(SIM_ObjectArray &objects)
    {
	// these should not persist between solves
	UT_ASSERT(!myDopIdLookup.entries());
	UT_ASSERT(!myGlueNetworks.entries());

	UT_HashTable objectnames;

	// construct mappings from objects to their position in myObjects
	for(int i = myObjects.entries() - 1; i >= 0; --i)
	{
	    int dopid = myObjects(i)->getDopId();
	    SIM_Object *obj = const_cast<SIM_Object *>(myEngine.getSimulationObjectFromId(dopid));

	    UT_Hash_Int64 hashkey(dopid);
	    UT_Thing thing((long)i);
	    myDopIdLookup.addSymbol(hashkey, thing);

	    UT_Hash_String hashkey2(obj->getName());
	    // multiple objects can have the same name
	    if(!objectnames.findSymbol(hashkey2, &thing))
	    {
		thing = (long)i;
		objectnames.addSymbol(hashkey2, thing);
	    }
	}

	// process RBD glue
	simGlueNetwork *network = 0;
	int n = objects.entries();
	for(int i = 0; i < n; ++i)
	{
	    SIM_Object *obj = objects(i);

	    UT_Hash_Int64 hashkey(obj->getObjectId());
	    UT_Thing thing;
	    if(!myDopIdLookup.findSymbol(hashkey, &thing))
		continue;

	    const RBD_State *rbdstate = SIM_DATA_GETCONST(*obj, "Position", RBD_State);
	    if(!rbdstate)
		continue;

	    UT_String parent;
	    rbdstate->getGlueObject(parent);
	    int index = indexFromObjectName(objectnames, parent);
	    if(index < 0)
		continue;

	    if(!network)
	    {
		network = new simGlueNetwork();
		myGlueNetworks.append(network);
	    }
	    network->addEdge((long)thing, index);
	}

	// process SIM_GlueNetworkRelationship items
	UT_HashTable known_glue_networks;
	for(int i = 0; i < n; ++i)
	{
	    SIM_Object *obj = objects(i);

	    SIM_ConstDataArray relation_data;
	    obj->filterConstRelationships(true,
                            SIM_DataFilterByType("SIM_GlueNetworkRelationship"),
			    relation_data);
	    for(int rel = 0; rel < relation_data.entries(); ++rel)
	    {
		SIM_Relationship *relation = const_cast<SIM_Relationship *>(SIM_DATA_CASTCONST(relation_data(rel), SIM_Relationship));
		if(!relation)
		    continue;

		// only process the glue network once
		UT_Hash_Ptr hashkey(relation);
		UT_Thing dummy;
		if(known_glue_networks.findSymbol(hashkey, &dummy))
		    continue;

		known_glue_networks.addSymbol(hashkey, dummy);

		SIM_GlueNetworkRelationship *glue_network =
			SIM_DATA_CAST(relation->getRelationshipTypeData(),
				      SIM_GlueNetworkRelationship);
		if(!glue_network)
		    continue;

		const SIM_Geometry *geo = SIM_DATA_GETCONST(*glue_network, SIM_GEOMETRY_DATANAME, SIM_Geometry);
		if(!geo)
		    continue;

		network = 0;
		GU_DetailHandleAutoReadLock gdl = geo->getGeometry();
		const GU_Detail *gdp = gdl.getGdp();
		const GA_IndexMap &map = gdp->getPointMap();

		UT_String name_attrib;
		glue_network->getNameAttrib(name_attrib);
		GA_ROAttributeRef name_attrib_ref = gdp->findStringTuple(GA_ATTRIB_POINT, name_attrib);
		bool name_is_string = name_attrib_ref.isValid();
		bool name_is_int = false;
		if(!name_is_string)
		{
		    name_attrib_ref = gdp->findIntTuple(GA_ATTRIB_POINT, name_attrib);
		    name_is_int = name_attrib_ref.isValid();
		}

		// if we do not have a valid attribute, then we assume that the
		// pieces map to the Geo_Point in the same order.
		for(GA_Index pr = gdp->primitives().entries() - 1; pr >= 0; --pr)
		{
		    const GEO_Primitive *prim = gdp->primitives()(pr);
		    int indexA = -1;
		    GA_Index ptA = GA_INVALID_INDEX;
		    for(int vtx = prim->getVertexCount() - 1; vtx >= 0; --vtx)
		    {
			// default behaviour is point ID represents DOP ID
			GEO_Point ppt(map, prim->getVertexElement(vtx).getPointOffset());
			GA_Index ptB = ppt.getNum();
			int indexB = -1;
			if(name_is_string)
			{
			    // name attribute specifies DOP object's name
			    indexB = indexFromObjectName(objectnames,
						ppt.getString(name_attrib_ref));
			}
			else
			{
			    // name attribute specifies a DOP ID
			    int dopid = name_is_int ? ppt.getValue<int>(name_attrib_ref) : static_cast<exint>(ptB);
			    if(dopid >= 0)
			    {
				UT_Hash_Int64 hashkey(dopid);
				UT_Thing thing;
				if(myDopIdLookup.findSymbol(hashkey, &thing))
				    indexB = (long)thing;
			    }
			}
			if(indexB < 0)
			    continue;
			if(indexA >= 0)
			{
			    if(!network)
			    {
				network = new simGlueNetwork(glue_network);
				myGlueNetworks.append(network);
			    }
			    network->addRelEdge(indexA, indexB, pr, ptA, ptB);
			}
			indexA = indexB;
			ptA = ptB;
		    }
		}
	    }
	}
    }

    // unlink an object from its glued chunk
    void unglue(simBulletObject *obj)
    {
	simBulletGluedChunk *chunk = obj->getGluedChunk();
	if(!chunk)
	    return;

	chunk->removePiece(obj->getPieceIndex());
	if(chunk->getNumPieces())
	    return;

	// remove chunk and shuffle remaining chunks to fill newly created hole
	simBulletGluedChunk *temp = myGluedChunks.removeLast();
	if(temp != chunk)
	{
	    int idx = chunk->getIndex();
	    temp->setIndex(idx);
	    myGluedChunks(idx) = temp;
	}
	delete chunk;
    }

    // apply impacts to objA
    void applyImpacts(simBulletObject *objA, simBulletObject *objB,
		      btPersistentManifold *manifold, bool reverse,
		      const SIM_ObjectArray &feedbacktoobjects)
    {
	const SIM_Time &t = myEngine.getSimulationTime();
	SIM_Object *obj = const_cast<SIM_Object *>(myEngine.getSimulationObjectFromId(objA->getDopId()));
	const SIM_BulletData *bulletdata = SIM_DATA_GETCONST(*obj,
						"Geometry/BulletData",
						SIM_BulletData);
	// only add create the impact if we want to add it
	bool apply = (bulletdata && bulletdata->getAddImpact());

	// add all the point impact to the impact data
	for(int i = manifold->getNumContacts() - 1; i >= 0; --i)
	{
	    const btManifoldPoint &pt = manifold->getContactPoint(i);
	    fpreal impulse = pt.getAppliedImpulse();
	    if(impulse)
	    {
		if(apply)
		{
		    SIM_Impacts *impact_data = 0;
		    if(!objA->isAffector())
		    {
			// create the impact data for the SIM_Object
			impact_data = SIM_DATA_CREATE(*obj,
				    SIM_NEWIMPACTS_DATANAME, SIM_Impacts,
				    SIM_DATA_RETURN_EXISTING);

		    }
		    else if(feedbacktoobjects.find(obj) >= 0)
		    {
			// create the impact data for the SIM_Object
			impact_data = SIM_DATA_CREATE(*obj,
				    SIM_NEWFEEDBACK_DATANAME, SIM_Impacts,
				    SIM_DATA_RETURN_EXISTING);
		    }
		    if(impact_data)
		    {
			btVector3 pos = pt.getPositionWorldOnB();
			btVector3 norm = pt.m_normalWorldOnB;
			if(reverse)
			    norm = -norm;

			impact_data->addPositionImpact(utVector3(pos),
				utVector3(norm), impulse,
				objB ? objB->getDopId() : -1, -1, -1, 0, 0, t, 0);
		    }
		}
		// accumulate impluse so we can update glue later
		objA->setImpulses(objA->getImpulses() + impulse);
	    }
	}
    }

    // method called when the Bullet solver advances by its timestep
    static void tickCallback(btDynamicsWorld *world, btScalar time_step)
    {
	simBulletState *state = static_cast<simBulletState *>(world->getWorldUserInfo());

	// notify all objects of the impluses applied to resolve collisions
	btCollisionDispatcher *dispatcher = state->myDispatcher;
	for(int i = dispatcher->getNumManifolds() - 1; i >= 0; --i)
	{
	    btPersistentManifold *manifold = dispatcher->getManifoldByIndexInternal(i);

	    // compute total impulse
	    btScalar impulse = 0;
	    for(int j = manifold->getNumContacts() - 1; j >= 0; --j)
		impulse += manifold->getContactPoint(j).getAppliedImpulse();

	    // don't do anything if there is no impulse
	    if(!impulse)
		continue;

	    const btManifoldPoint &pt = manifold->getContactPoint(0);
	    simBulletObject *objA = simObjectFromShapeIdx(pt.m_index0,
			static_cast<btRigidBody *>(manifold->getBody0()));

	    simBulletObject *objB = simObjectFromShapeIdx(pt.m_index1,
			static_cast<btRigidBody *>(manifold->getBody1()));
		
	    // if the shape is a concave shape, then the index id would be -1,
	    // in which case we cannot determine which piece is colliding.
	    if(objA)
	    {
		// now we have objA, we can actually get the SIM_Object that is
		// suppose to have the impact data
		state->applyImpacts(objA, objB, manifold, false,
				    *state->myFeedbackToObjects);
	    }

	    if(objB)
	    {
		// now we have objB, we can actually get the SIM_Object that is
		// suppose to have the impact data
		state->applyImpacts(objB, objA, manifold, true,
				    *state->myFeedbackToObjects);
	    }
	}
    }

    // number references to this object
    int myRefCount;

    // the Bullet simulation framework
    btDefaultCollisionConfiguration *myCollisionConfiguration;
    btCollisionDispatcher *myDispatcher;
    btBroadphaseInterface *myBroadphase;
    btSequentialImpulseConstraintSolver *mySolver;
    btDiscreteDynamicsWorld *myDynamicsWorld;

    // the source of the DOP simulation and simulation time currently
    // represented in Bullet
    SIM_Engine &myEngine;
    SIM_Time myTime;

    // all objects in the Bullet simulation
    UT_PtrArray<simBulletObject *> myObjects;
    // all glued chunks in the Bullet simulation
    UT_PtrArray<simBulletGluedChunk *> myGluedChunks;

    //
    // the following are temporary values used when advancing the timestep and
    // should be empty between solves
    //

    // mapping from a DOP ID to the index of the corresponding simBulletObject
    UT_HashTable myDopIdLookup;

    // uniform representation of all glue networks
    UT_PtrArray<simGlueNetwork *> myGlueNetworks;

    // all constraints in the Bullet simulation
    UT_PtrArray<simBulletConstraint *> myConstraints;

    // Feedback affectors
    const SIM_ObjectArray *myFeedbackToObjects;
};

// constructor
SIM_SolverBulletHolladay::SIM_SolverBulletHolladay(const SIM_DataFactory *factory)
    : BaseClass(factory), SIM_OptionsUser(this), myState(0)
{
}

// destructor
SIM_SolverBulletHolladay::~SIM_SolverBulletHolladay()
{
    if(myState)
	myState->unref();
}

// templates used to create DOP nodes for creating instances of this solver in
// DOP networks
const SIM_DopDescription *
SIM_SolverBulletHolladay::getSolverBulletDopDescription()
{
    static PRM_Range	simSubstepsRange(PRM_RANGE_UI, 1, PRM_RANGE_UI, 100);
    static PRM_Range	simIterRange(PRM_RANGE_UI, 1, PRM_RANGE_UI, 100);
    static PRM_Range	simSIPTRange(PRM_RANGE_UI, -1, PRM_RANGE_UI, 1);

    static PRM_Name	simSubsteps(SIM_NAME_SUBSTEPS, "Number of Substeps");
    static PRM_Name	simNumIter(SIM_NAME_ITERATION, "Constraint Iterations");
    static PRM_Name	simSplitImpulse(SIM_NAME_SPLITIMPULSE, "Split Impulse");
    static PRM_Name	simSIPT(SIM_NAME_SIPT, "Penetration Threshold");

    static PRM_Default  defNegPointZeroTwo(-0.02f);

    static PRM_Template	theTemplates[] =
    {
	PRM_Template(PRM_INT_J,	1, &simSubsteps, PRMtenDefaults, 0,
				&simSubstepsRange),
	PRM_Template(PRM_INT_J,	1, &simNumIter, PRMtenDefaults, 0,
				&simIterRange),
	PRM_Template(PRM_TOGGLE_J, 1, &simSplitImpulse, PRMzeroDefaults),
	PRM_Template(PRM_FLT_J,	1, &simSIPT, &defNegPointZeroTwo, 0,
				&simSIPTRange),
	PRM_Template()
    };

    static SIM_DopDescription theDopDescription(true,
						"bulletrbdsolverholladay",
						"Bullet Solver Holladay",
						"Solver",
						classname(),
						theTemplates);

    return &theDopDescription;
}

// instances of the solver at different simulation frames share the same
// Bullet simulation
void
SIM_SolverBulletHolladay::makeEqualSubclass(const SIM_Data *src)
{
    BaseClass::makeEqualSubclass(src);
    const SIM_SolverBulletHolladay *s = SIM_DATA_CASTCONST(src, SIM_SolverBulletHolladay);
    if(s)
    {
	simBulletState *state = s->myState;
	if(state)
	    state->ref();
	if(myState)
	    myState->unref();
	myState = state;
    }
}

// called when the simulation advances by a time step
// 'objects' contains all objects we need to process
SIM_Solver::SIM_Result
SIM_SolverBulletHolladay::solveObjectsSubclass(SIM_Engine &engine,
	SIM_ObjectArray &objects,
	SIM_ObjectArray &newobjects,
	SIM_ObjectArray &feedbacktoobjects,
	const SIM_Time &timestep)
{
    const SIM_Time &t = engine.getSimulationTime();

    // update guide geometry for newly created objects, these will be simulated
    // on the next time step
    for(int i = newobjects.entries() - 1; i >= 0; --i)
    {
	SIM_Object *obj = newobjects(i);
	simAutofit(*obj);
	SIM_ConstraintIterator::initConstraints(*obj, t);
    }

    for(int i = objects.entries() - 1; i >= 0; --i)
	SIM_ConstraintIterator::initConstraints(*objects(i), t);

    // initialise simulation environment
    if(!myState)
	myState = new simBulletState(engine);
    else if(!SYSisEqual(myState->getSimulationTime() + timestep, t))
    {
	// reset to ensure Bullet has not keep cached state that is no longer
	// valid
	myState->reset();
    }

    // export simulation state to Bullet
    myState->setSolverParameters(getNumIteration(), getSplitImpulse(),
				 getSplitImpulsePenetrationThreshold());
    myState->updateFromObjects(this, objects);
    myState->glueObjects(objects);
	myState->saveTransforms(objects);	// EDIT // This function saves out the velocities of all glue objects to be able to restore velocities if pieces break off
    myState->applyForces(objects, timestep);
    myState->applyConstraints(objects);
	
	myState->performDiscreteCollisionDetection(objects, timestep);		// EDIT // Compute all objects' collisions without changing their positions/impulses
	myState->removeContactedGlueObjects(objects);						// EDIT // Remove any glue pieces that impacted something with an impulse beyond their glue strength
	myState->updateGlue(objects, timestep);			// EDIT // Update the glue
	
	myState->resetTransforms( objects );	// EDIT // This function restores the previous state of all the rigid bodies
	myState->glueObjects(objects);		// EDIT // Reglue the glue objects
	myState->applyForces(objects, timestep);	// EDIT // Reapply the forces
	myState->stepSimulation(t, timestep, getSubsteps(), feedbacktoobjects);
	myState->removeConstraints(objects, timestep);
	myState->updateObjects(objects, timestep);
	myState->updateGlue(objects, timestep);
	
    for(int i = objects.entries() - 1; i >= 0; --i)
	objects(i)->preserveImpacts(engine);
    for(int i = feedbacktoobjects.entries() - 1; i >= 0; --i)
	feedbacktoobjects(i)->preserveImpacts(engine);
	
	// EDIT // Resetting myState because for some reason myState is getting
	//           recreated each frame (put a print statement in ~simBulletState()),
	//           causing memory leaks because all the simBulletObjects are never deleted
	//           in the cleanup, and when reset() is called in the constructor, there
	//           are no simBulletObjects to delete.
	myState->reset();
	//////////

    return SIM_Solver::SIM_SOLVER_SUCCESS;
}

// register our operators
void
initializeSIM(void *)
{
    IMPLEMENT_DATAFACTORY(SIM_SolverBulletHolladay);
    IMPLEMENT_DATAFACTORY(SIM_BulletData);
    IMPLEMENT_DATAFACTORY(SIM_ConRelConeTwist);
}


























const SIM_DopDescription*
SIM_BulletData::getBulletDataDopDescription()
{
    static PRM_Name theGeoRep(SIM_NAME_GEO_REP, "Geometry Representation");
    static PRM_Name theGeoConvex(SIM_NAME_GEO_CONVEX,
				 "Polygons As Convex Hulls");
    static PRM_Name thePrimAutofit(SIM_NAME_PRIM_AUTOFIT,
		"AutoFit Primitive Boxes, Capsules, or Spheres to Geometry");
    static PRM_Name thePrimT(SIM_NAME_PRIM_T, "Position");
    static PRM_Name thePrimR(SIM_NAME_PRIM_R, "Rotation");
    static PRM_Name thePrimS(SIM_NAME_PRIM_S, "Box Size");
    static PRM_Name thePrimRadius(SIM_NAME_PRIM_RADIUS, "Radius");
    static PRM_Name thePrimLength(SIM_NAME_PRIM_LENGTH, "Capsule Length");
    static PRM_Name theAdjGeo(SIM_NAME_ADJUST_GEOMETRY, "Adjust Geometry");
    static PRM_Name theAdjFactor(SIM_NAME_ADJUST_FACTOR, "Adjustment Factor");
    static PRM_Name theCollisionMargin(SIM_NAME_COLLISION_MARGIN,
				       "Collision Padding");
    static PRM_Name theAddImpact(SIM_NAME_ADDIMPACT, "Add Impact Data");
    static PRM_Name theDeactivate(SIM_NAME_DEACTIVATE, "Enable Sleeping");
    static PRM_Name theOptimized(SIM_NAME_OPTIMIZED, "Optimized");

    static PRM_Name theGeoRepNames[] =
    {
	PRM_Name(GEO_REP_AS_IS,		"As Is"),
	PRM_Name(GEO_REP_BOX,		"Box"),
	PRM_Name(GEO_REP_CAPSULE,	"Capsule"),
	PRM_Name(GEO_REP_COMPOUND,	"Compound"),
	PRM_Name(GEO_REP_SPHERE,	"Sphere"),
	PRM_Name(0)
    };
    static PRM_ChoiceList theGeoRepNamesMenu((PRM_ChoiceListType)
						(PRM_CHOICELIST_REPLACE
						| PRM_CHOICELIST_EXCLUSIVE ),
						&(theGeoRepNames[0]) );

    static PRM_Conditional disAutoFit(
		    "{ bullet_georep == compound } "
		    "{ bullet_georep == as-is }");
    static PRM_Conditional disConvexHulls(
		    "{ bullet_georep != as-is }");
    static PRM_Conditional disPrimT(
		    "{ bullet_autofit == 1 } { bullet_georep == compound } "
		    "{ bullet_georep == as-is }");
    static PRM_Conditional disPrimR(
		    "{ bullet_autofit == 1 } { bullet_georep == compound } "
		    "{ bullet_georep == as-is }");
    static PRM_Conditional disPrimS(
		    "{ bullet_autofit == 1 } { bullet_georep == sphere } "
		    "{ bullet_georep == as-is } { bullet_georep == compound } "
		    "{ bullet_georep == capsule }");
    static PRM_Conditional disRadius(
		    "{ bullet_autofit == 1 } { bullet_georep == as-is } "
		    "{ bullet_georep == compound } { bullet_georep == box }");
    static PRM_Conditional disCapsuleLength(
		    "{ bullet_autofit == 1 } { bullet_georep != capsule }");
    static PRM_Conditional disAdjGeo(
		    "{ bullet_georep == sphere } { bullet_georep == box } "
		    "{ bullet_georep == capsule }");
    static PRM_Conditional disAdjFactor(
		    "{ bullet_georep == sphere } { bullet_georep == box } "
		    "{ bullet_georep == capsule } "
		    "{ bullet_adjust_geometry == 0 }");

    static PRM_Default defGeoRep(0, GEO_REP_AS_IS);
    static PRM_Default defCollisionMargin(0.02);
    static PRM_Default defRedVec[] =
    {
	PRM_Default(1), PRM_Default(0), PRM_Default(0)
    };

    static PRM_Range geoRepRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 5);
    static PRM_Range geoGeoTriRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 1);
    static PRM_Range primRadiusRange(PRM_RANGE_UI, 0.1, PRM_RANGE_UI, 5);
    static PRM_Range primLengthRange(PRM_RANGE_UI, 0.1, PRM_RANGE_UI, 5);
    static PRM_Range collisionMarginRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 0.5);
    static PRM_Range adjFactorRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 5);

    static PRM_Template theTemplates[] =
    {
	PRM_Template(PRM_STRING,    1, &theGeoRep, &defGeoRep,
				    &theGeoRepNamesMenu),
	PRM_Template(PRM_TOGGLE,    1, &theGeoConvex, PRMoneDefaults, 0,
				    0, 0, 0, 1, 0, &disConvexHulls),
	PRM_Template(PRM_TOGGLE,    1, &thePrimAutofit, PRMoneDefaults, 0,
				    &geoGeoTriRange, 0, 0, 1, 0, &disAutoFit),
	PRM_Template(PRM_XYZ,	    3, &thePrimT, PRMzeroDefaults,
				    0, 0, 0, 0, 1, 0, &disPrimT),
	PRM_Template(PRM_XYZ,	    3, &thePrimR, PRMzeroDefaults,
				    0, 0, 0, 0, 1, 0, &disPrimR),
	PRM_Template(PRM_XYZ,	    3, &thePrimS, PRMoneDefaults,
				    0, 0, 0, 0, 1, 0, &disPrimS),
	PRM_Template(PRM_FLT,	    1, &thePrimRadius, PRMoneDefaults, 0,
				    &primRadiusRange, 0, 0, 1, 0, &disRadius),
	PRM_Template(PRM_FLT,	    1, &thePrimLength, PRMoneDefaults, 0,
				    &primLengthRange, 0, 0, 1, 0,
				    &disCapsuleLength),
	PRM_Template(PRM_TOGGLE,    1, &theAdjGeo, PRMoneDefaults,
				    0, 0, 0, 0, 1, 0, &disAdjGeo),
	PRM_Template(PRM_FLT,	    1, &theAdjFactor, PRMoneDefaults, 0,
				    &adjFactorRange, 0, 0, 1, 0, &disAdjFactor),
	PRM_Template(PRM_FLT,	    1, &theCollisionMargin, &defCollisionMargin,
				    0, &collisionMarginRange),
	PRM_Template(PRM_TOGGLE,    1, &theAddImpact, PRMoneDefaults),
	PRM_Template(PRM_TOGGLE,    1, &theDeactivate, PRMoneDefaults),
	PRM_Template(PRM_TOGGLE,    1, &theOptimized, PRMoneDefaults),
	PRM_Template()
    };
    static PRM_Template theGuideTemplates[] =
    {
	PRM_Template(PRM_TOGGLE,    1, &SIMshowguideName, PRMzeroDefaults),
	PRM_Template(PRM_RGB,	    3, &SIMcolorName, defRedVec, 0,
				    &PRMunitRange),
	PRM_Template()
    };

    static SIM_DopDescription theDopDescription(true,
						"bulletdata",
						"Bullet Data",
						"Geometry/BulletData",
						classname(),
						theTemplates);
    theDopDescription.setGuideTemplates(theGuideTemplates);

    return &theDopDescription;
}

SIM_BulletData::SIM_BulletData(const SIM_DataFactory *factory)
    : BaseClass(factory), SIM_OptionsUser(this)
{
}

SIM_BulletData::~SIM_BulletData()
{
}

void
SIM_BulletData::autofit(const SIM_Geometry &simgeo)
{
    // clear autofit flag
    setAutofit(false);

    UT_String geo_rep;
    getGeoRep(geo_rep);
    if (geo_rep == GEO_REP_AS_IS || geo_rep == GEO_REP_COMPOUND)
    {
	// do nothing
	return;
    }

    GU_DetailHandleAutoReadLock gdl(simgeo.getGeometry());
    const GU_Detail *gdp = gdl.getGdp();
    UT_ASSERT(gdp);
    GU_Detail simgdp(gdp);

    UT_Matrix4D geoXform;
    simgeo.getTransform(geoXform);
    UT_Matrix4 geo_xform(geoXform);
    simgdp.transform(geo_xform);

    if (geo_rep == GEO_REP_BOX)
    {
	UT_OBBoxD obb;
	simgdp.getOBBoxForPoints(0, obb);

	setPrimT(obb.getCenter());
	setPrimR(180 / M_PI * obb.getRotation());
        setPrimS(obb.getRadii());
    }
    else if (geo_rep == GEO_REP_SPHERE)
    {
	UT_BoundingSphere bSphere;
	simgdp.getBSphere(&bSphere);

	setPrimRadius(bSphere.getRadius());
	setPrimT(bSphere.getCenter());
	setPrimR(UT_Vector3(0, 0, 0));
    }
    else if (geo_rep == GEO_REP_CAPSULE)
    {
	UT_OBBoxD obb;
	simgdp.getOBBoxForPoints(0, obb);

	UT_Vector3D rotate = obb.getRotation();
	UT_Vector3D size = obb.getRadii();

	fpreal  length;
	fpreal  radius;
	fpreal  xy_diff = fabs(size[0] - size[1]);
	fpreal  yz_diff = fabs(size[1] - size[2]);
	fpreal  xz_diff = fabs(size[2] - size[0]);

	if (xy_diff <= xz_diff && xy_diff <= yz_diff)
	{
	    // xy_diff is the smallest, z axis is the principle axis
	    UT_XformOrder order;
	    order.mainOrder(UT_XformOrder::RTS);
	    order.rotOrder(UT_XformOrder::XYZ);

	    UT_Matrix3D mat(1.0);
	    mat.rotate(rotate.x(), rotate.y(), rotate.z(), UT_XformOrder());
	    mat.prerotate(M_PI_2, 0, 0, order);
	    mat.crack(rotate, order);

	    length = size[2];
	    radius = sqrt(size[0]*size[0] + size[1]*size[1]);
	}
	else if (xz_diff <= xy_diff && xz_diff <= yz_diff)
	{
	    // xz_diff is the smallest, y axis is the principle axis
	    length = size[1];
	    radius = sqrt(size[0]*size[0] + size[2]*size[2]);
	}
	else
	{
	    // yz_diff is the smallest, x axis is the principle axis
	    UT_XformOrder order;
	    order.mainOrder(UT_XformOrder::RTS);
	    order.rotOrder(UT_XformOrder::XYZ);

	    UT_Matrix3D mat(1.0);
	    mat.rotate(rotate.x(), rotate.y(), rotate.z(), UT_XformOrder());
	    mat.prerotate(0, 0, M_PI_2, order);
	    mat.crack(rotate, order);

	    length = size[0];
	    radius = sqrt(size[1]*size[1] + size[1]*size[1]);
	}

	length = ((length-radius)*2<0) ? 0:(length-radius)*2;

	setPrimT(obb.getCenter());
	setPrimR(180 / M_PI * rotate);
	setPrimLength(length);
	setPrimRadius(radius);
    }
}

SIM_Guide *
SIM_BulletData::createGuideObjectSubclass() const
{
    return new SIM_GuideTimeDep<SIM_GuidePerObject>(this);
}

void
SIM_BulletData::buildGuideGeometrySubclass(const SIM_RootData &,
					   const SIM_Options &options,
					   const GU_DetailHandle &gdh,
					   UT_Matrix4D *xform,
					   const SIM_Time &) const
{
    UT_String geo_rep;
    getGeoRep(geo_rep);

    if (geo_rep == GEO_REP_AS_IS || geo_rep == GEO_REP_COMPOUND)
    {
	// do not do anything for as is
	return;
    }

    UT_XformOrder order;
    order.mainOrder(UT_XformOrder::RTS);
    order.rotOrder(UT_XformOrder::XYZ);

    UT_Vector3D prim_trans = getPrimTD();
    UT_Vector3D prim_rot = getPrimRD();

    if(!gdh.isNull())
    {
	GU_DetailHandleAutoWriteLock gdl(gdh);
	GU_Detail *gdp = gdl.getGdp();
	if (geo_rep == GEO_REP_BOX)
	{
	    UT_Vector3D prim_scale = getPrimSD();
	    gdp->cube(-prim_scale.x(), prim_scale.x(),
		      -prim_scale.y(), prim_scale.y(),
		      -prim_scale.z(), prim_scale.z(),
		       1, 1, 1 );
	}
	else if (geo_rep == GEO_REP_SPHERE)
	{
	    fpreal prim_radius = getPrimRadius();

	    GU_PrimSphereParms parms;
	    parms.gdp = gdp;
	    parms.xform.identity();
	    parms.xform.scale(prim_radius, prim_radius, prim_radius);

	    GU_PrimSphere::build(parms);
	}
	else if (geo_rep == GEO_REP_CAPSULE)
	{
	    fpreal prim_radius = getPrimRadius();
	    fpreal prim_length = getPrimLength();

	    GU_PrimTubeParms t_parms;
	    GU_CapOptions caps;
	    t_parms.gdp = gdp;
	    t_parms.cols = 4;
	    t_parms.imperfect = true;
	    t_parms.type = GEO_PATCH_COLS;
	    t_parms.xform.scale(prim_radius, prim_length, prim_radius);
	    GU_PrimTube::build(t_parms, caps, GEO_PRIMPOLY);

	    GU_PrimCircleParms c_parms;
	    c_parms.gdp = gdp;
	    c_parms.type = GU_CIRCLE_OPEN_ARC;
	    c_parms.imperfect = true;
	    c_parms.divisions = 10;
	    c_parms.order = 4;
	    c_parms.xform.scale(prim_radius, prim_radius, prim_radius);

	    c_parms.beginAngle = 0;
	    c_parms.endAngle = M_PI;
	    c_parms.xform.translate(0, prim_length/2, 0);
	    GU_PrimCircle::build(c_parms, GEO_PRIMNURBCURVE);

	    c_parms.beginAngle = M_PI;
	    c_parms.endAngle = 2*M_PI;
	    c_parms.xform.translate(0, -prim_length, 0);
	    GU_PrimCircle::build(c_parms, GEO_PRIMNURBCURVE);


	    c_parms.xform.rotate(0.0, M_PI/2, 0.0, order);
	    c_parms.beginAngle = 0;
	    c_parms.endAngle = M_PI;
	    c_parms.xform.translate(0, prim_length, 0);
	    GU_PrimCircle::build(c_parms, GEO_PRIMNURBCURVE);

	    c_parms.beginAngle = M_PI;
	    c_parms.endAngle = 2*M_PI;
	    c_parms.xform.translate(0, -prim_length, 0);
	    GU_PrimCircle::build(c_parms, GEO_PRIMNURBCURVE);

	    c_parms.beginAngle = 0;
	    c_parms.endAngle = 2*M_PI;
	    c_parms.xform.identity();
	    c_parms.xform.scale(prim_radius, prim_radius, prim_radius);

	    c_parms.xform.rotate(M_PI/2, 0.0, 0.0, order);
	    c_parms.xform.translate(0, prim_length/2, 0);
	    GU_PrimCircle::build(c_parms, GEO_PRIMNURBCURVE);
	    c_parms.xform.translate(0, -prim_length, 0);
	    GU_PrimCircle::build(c_parms, GEO_PRIMNURBCURVE);

	    prim_rot = getPrimRD();
	}

	UT_Vector3 color(1, 1, 1);

	GA_WOAttributeRef color_att = gdp->addDiffuseAttribute(GEO_PRIMITIVE_DICT);

	if (options.hasOption(SIMcolorName.getToken()))
	    color = options.getOptionV3(SIMcolorName.getToken());


	int nprims = gdp->primitives().entries();
	for (int i = 0; i < nprims; ++i)
	    gdp->primitives()(i)->setValue<UT_Vector3>(color_att, color);
    }

    if(xform)
    {
	prim_rot = M_PI / 180 * prim_rot; // convert to radian
	xform->rotate(prim_rot.x(), prim_rot.y(), prim_rot.z(), order);
	xform->translate(prim_trans.x(), prim_trans.y(), prim_trans.z());
    }
}
























const SIM_DopDescription *
SIM_ConRelConeTwist::getConRelConeTwistDopDescription()
{
    static PRM_Name	theSwingSpan1(SIM_NAME_SWING1, "Swing Span 1");
    static PRM_Name	theSwingSpan2(SIM_NAME_SWING2, "Swing Span 2");
    static PRM_Name	theTwistSpan(SIM_NAME_TWIST, "Twist Span");
    static PRM_Name	theSoftness(SIM_NAME_SOFTNESS, "Softness");
    static PRM_Name	theBaisFactor(SIM_NAME_BIAS_FACTOR, "Bias Factor");
    static PRM_Name	theRelaxation(SIM_NAME_RELAXATION_FACTOR, "Relaxation Factor");
    static PRM_Name	theTwistAxis(SIM_NAME_TWIST_AXIS, "Twist Axis");
    
    static PRM_Default	defPi(M_PI);
    static PRM_Default	defPoint3(0.3);
    static PRM_Default	defRedVec[] =
    {
	PRM_Default(1), PRM_Default(0), PRM_Default(0)
    };
    static PRM_Default	defAxis[] =
    {
	PRM_Default(1), PRM_Default(0), PRM_Default(0)
    };

    static PRM_Template	theTemplates[] = {
	PRM_Template(PRM_FLT_J, 1, &theSwingSpan1, &defPi),
	PRM_Template(PRM_FLT_J, 1, &theSwingSpan2, &defPi),
	PRM_Template(PRM_FLT_J, 1, &theTwistSpan, &defPi),
	PRM_Template(PRM_FLT_J, 1, &theSoftness, PRMoneDefaults),
	PRM_Template(PRM_FLT_J, 1, &theBaisFactor, &defPoint3),
	PRM_Template(PRM_FLT_J, 1, &theRelaxation, PRMoneDefaults),
	PRM_Template(PRM_XYZ, 3, &theTwistAxis, defAxis, 0),
	PRM_Template()
    };

    static PRM_Template	theGuideTemplates[] = {
	PRM_Template(PRM_TOGGLE, 1, &SIMshowguideName, PRMzeroDefaults),
	PRM_Template(PRM_RGB, 3, &SIMcolorName, defRedVec, 0, &PRMunitRange),
	PRM_Template()
    };

    static SIM_DopDescription theDopDescription(true, "conetwistconrel",
					"Cone Twist Constraint Relationship",
					"ConRelConeTwist",
					classname(),
					theTemplates);

    theDopDescription.setGuideTemplates(theGuideTemplates);

    return &theDopDescription;
}

SIM_ConRelConeTwist::SIM_ConRelConeTwist(const SIM_DataFactory *factory)
    : BaseClass(factory), SIM_OptionsUser(this)
{
}

SIM_ConRelConeTwist::~SIM_ConRelConeTwist()
{
}















