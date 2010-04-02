// 
// Open Source Bullet Solver
//


#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h>
#include <BulletCollision/CollisionShapes/shCompoundSphereShape.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#include <UT/UT_DSOVersion.h>
#include <UT/UT_XformOrder.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_SDF.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Options.h>
#include <SIM/SIM_PhysicalParms.h>
#include <SIM/SIM_ActiveValue.h>
#include <SIM/SIM_Data.h>
#include <SIM/SIM_Query.h>
#include <SIM/SIM_Utils.h>
#include <RBD/RBD_State.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Force.h>
#include <SIM/SIM_ForcePoint.h>
#include <SIM/SIM_ForceUniform.h>
#include <SIM/SIM_ForceGravity.h>
#include <SIM/SIM_Constraint.h>
#include <SIM/SIM_ConAnchorObjSpacePos.h>
#include <SIM/SIM_ConAnchorWorldSpacePos.h>
#include <SIM/SIM_ConAnchor.h>
#include <SIM/SIM_ConRel.h>
#include <SIMZ/SIM_PopGeometry.h>
#include <GEO/GEO_Detail.h>
#include <GEO/GEO_PrimSphere.h>
#include <UT/UT_Quaternion.h>

#include "SIM_SnowSolverBullet.h"


#ifdef WIN32		// this prevents warnings when dependencies built
#include <windows.h>
#endif

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)


btTransform convertTobtTransform(const UT_DMatrix4 &inMat)
{
	UT_Matrix3 rotmatrix;
	inMat.extractRotate( rotmatrix );
	btMatrix3x3 btRotMatrix( rotmatrix(0,0), rotmatrix(0,1), rotmatrix(0,2),
				 rotmatrix(1,0), rotmatrix(1,1), rotmatrix(1,2),
				 rotmatrix(2,0), rotmatrix(2,1), rotmatrix(2,2) );
	UT_Vector3  translates;
	inMat.getTranslates( translates );
	btTransform btTrans;
	btTrans.setOrigin( btVector3( translates.x(),translates.y(),translates.z() ) );
	btTrans.setBasis( btRotMatrix );
	//return btTransform( btRotMatrix, btVector3( translates.x(),translates.y(),translates.z() ) );
	return btTrans;
}

//Constructor
SIM_SnowSolverBullet::SIM_SnowSolverBullet(const SIM_DataFactory *factory)
: BaseClass(factory), SIM_OptionsUser(this), state(0)
{
}

//Destructor
SIM_SnowSolverBullet::~SIM_SnowSolverBullet()
{
	if (state)
		state->removeReference();
}


//UI Param Def
const SIM_DopDescription *
SIM_SnowSolverBullet::getSolverBulletDopDescription()
{
	static PRM_Template	theTemplates[] = {
			PRM_Template()
	};

	static SIM_DopDescription   theDopDescription(true,
		"bulletsnowsolver",
		"Bullet Snow Solver",
		"Solver",
		classname(),
		theTemplates);

	return &theDopDescription;
}



// -------------------------------------------------------------------
//	The solveObjectsSubclass is called once per simulation step. 
//  The objects array has all of the objects you are to solve for.
SIM_Solver::SIM_Result SIM_SnowSolverBullet::solveObjectsSubclass(SIM_Engine &engine,
	SIM_ObjectArray &objects,
	SIM_ObjectArray &newobjects,
	SIM_ObjectArray &feedbacktoobjects,
	const SIM_Time &timestep)
{
	int i;
	UT_Vector3 p, v, w;
	UT_Quaternion rot;
	std::map< int, bulletBody >::iterator bodyIt;
	// ADDED BY SRH 2010-03-31 //
	std::map< int, bulletBody >::iterator affectorIt;
	// *********************** //
	btTransform btrans;
	int totalDopObjects = engine.getNumSimulationObjects();
	int totalUpdateObjects = engine.getNumSimulationObjects();
	float currTime = engine.getSimulationTime();
	
	// ADDED BY SRH 2010-03-30 FOR AFFECTOR CODE//
	SIM_ObjectArray collisionAffectors;
	const SIM_Geometry  *geometry = 0;
	// *********************** //


	if( !state )
	{
		//cerr<<"creating new state"<<endl;
		state = new SIM_SnowSolverBulletState();
		state->addReference();
	}
	
	// ADDED BY SRH 2010-03-29 //
	//int numRelationships = engine.getNumRelationships();
	//cout << "Num relationships = " << numRelationships << endl;
	// *********************** //
	
	//cout<<"remove any that have been deleted from dops in system:"<<state->m_dynamicsWorld<<endl;
	removeDeadBodies( engine );	

	////////Loop over all objects 
	//////for(i = 0; i < totalDopObjects; i++)
	// ADDED BY SRH 2010-04-01 - Now, instead of looping over all objects in the engine
	//                             it, only loops over objects directly connected to this Bullet Solver node. //
	// First we add/update all objects for which we'll be solving using Bullet.
    //   (Objects using other solvers are not included here.)
    int numBulletObjects = objects.entries();	// "objects" was passed in as input to this function
    for ( i = 0; i < numBulletObjects; i++ )
    {
    // *********************** //
		//Get current dop sim object
		//SIM_Object *currObject = (SIM_Object *)engine.getSimulationObject(i);
		SIM_Object *currObject = (SIM_Object *)objects(i);
		
		//Check if this body has been added already, if not, add it.
		//Bullet bodies are stored in a map, the key is the dopObjectId
		bodyIt = state->m_bulletBodies->find( currObject->getObjectId() );
		
		//This is a new body (that is, it doesn't match any key in the map), make a bullet body to match
		if(bodyIt == state->m_bulletBodies->end())
			bodyIt = addBulletBody( currObject );
		
		if(bodyIt != state->m_bulletBodies->end())
		{
			// Pull the DOPs subdata and set the Bullet Transforms with it.			
			RBD_State *rbdstate = SIM_DATA_GET(*currObject, "Position", RBD_State);
			//SIM_SnowBulletData *bulletstate = SIM_DATA_GET(*currObject, "Bullet Data", SIM_SnowBulletData);

			// Pull initial Position & Rotation from subdata and set bullet transform
			p = rbdstate->getPosition();
			rot =	rbdstate->getOrientation();
			btrans.setRotation( btQuaternion( rot.x(), rot.y(), rot.z(), rot.w() ) );
			btrans.setOrigin( btVector3(p[0],p[1],p[2]) );
			(bodyIt->second.bodyId)->getMotionState()->setWorldTransform(btrans);
			
			if( rbdstate->getMass() > 0 )
			{
				//Velocity
				v = rbdstate->getVelocity();			
				(bodyIt->second.bodyId)->setLinearVelocity( btVector3(v[0],v[1],v[2]) ); 
				//Angular Velocity
				w = rbdstate->getAngularVelocity();
				(bodyIt->second.bodyId)->setAngularVelocity( btVector3(w[0],w[1],w[2]) ); 
			}
			//Add forces and such from the rest of the subdata
			processSubData(currObject, bodyIt->second);
		}	
	}  // for each object(i)
	
	// ADDED BY SRH 2010-04-01 *************************************** //
	//   Based on the affector for-loop in the ODESolver code!  Thanks!
	//   However, this is simplified and CURRENTLY ONLY WORKS WITH STATIC (non-dynamic) AFFECTOR OBJECTS!
	for ( i = 0; i < numBulletObjects; i++ )
	{
		SIM_ColliderInfoArray colliders;
		SIM_Object *currAffector;
		//ObjectMap *odeaffectors = worlddata->getOdeAffectors();
		
		SIM_Object *currObject = (SIM_Object *)objects(i);
		
		currObject->getColliderInfo( colliders );
		bodyIt = state->m_bulletBodies->find( currObject->getObjectId() );
		for (int a = 0; a < colliders.entries(); a++)
		{
			currAffector = colliders(a).getAffector();
			affectorIt = state->m_bulletAffectors->find( currAffector->getObjectId() );
			if ( affectorIt == state->m_bulletAffectors->end() )
			{
				bodyIt = state->m_bulletBodies->find( currAffector->getObjectId() );
				if ( bodyIt != state->m_bulletBodies->end() )
				{
				    // This object has already been added as an ODE body
				    // Make sure collisions are detected appropriately
				}
				else
				{
				    // Add this affector object to the sim AS A BULLET BODY (not with addAffector(), but with addBulletBody())
				    //   Therefore, objects connected to this SIM_SnowSolverBullet node and objects
				    //   affecting those objects are treated equally, except the data (position, etc.)
				    //   is not updated for the affectors each frame, only when it is first added.
				    //   ***This will need to be fixed in the future to take into affect dynamic affectors.***
				    affectorIt = addBulletBody( currAffector );
				}  // else
			}  // if
		}  // for each affector(a) of the current object
	}  // for each object(i), look for additional affectors
	// *************************************************************** //

	//Run Sim and update DOPS not on init frame
	if(currTime > 0) 
	{
		//cout<<"step world"<<endl;
		
		// THIS IS WHERE IT ALL HAPPENS IN BULLET!!!!!! //
		state->m_dynamicsWorld->stepSimulation( 1/24.f, 10 );
		// ******************************************** //

		//Iterate over active objects and update the dop geometry	
		//for	(i = 0;	i <	totalUpdateObjects; i++)
		for ( i = 0; i < numBulletObjects; i++ )
		{
			//Get current sim object
			//SIM_Object *currObject = (SIM_Object *)engine.getSimulationObject(i);
			SIM_Object *currObject = (SIM_Object *)objects(i);
			
			/*UT_String name = currObject->getName();
			if ( name == "box_object1" ) {
				continue;
			}*/
		
			//Get State for Curr Object
			RBD_State	*rbdstate;
			rbdstate =  SIM_DATA_GET(*currObject, "Position", RBD_State);
			//SIM_SnowBulletData *bulletstate = SIM_DATA_GET(*currObject, "Bullet Data", SIM_SnowBulletData);
	
			//Get the matching bullet body in map
			bodyIt = state->m_bulletBodies->find( currObject->getObjectId() );
		
			if(bodyIt != state->m_bulletBodies->end())
			{	
				// Get the Bullet transform and update the DOPs subdata with it.
				(bodyIt->second.bodyId)->getMotionState()->getWorldTransform(btrans);
				// position
				UT_Vector3 p = UT_Vector3(
					btrans.getOrigin().getX(), btrans.getOrigin().getY(), btrans.getOrigin().getZ() );
				rbdstate->setPosition( p );
				// rotation    		
				UT_Quaternion rot = UT_Quaternion( btrans.getRotation().getX(), btrans.getRotation().getY(),
					btrans.getRotation().getZ(), btrans.getRotation().getW() );
				rbdstate->setOrientation( rot );

				// Calculate mass and inertia
				btScalar mass = 1.0f;
				if( currObject->getIsStatic() )
					mass = 0;
				else
				{
					mass = rbdstate->getMass();
					//btVector3 fallInertia(0,0,0);
					//(bodyIt->second.bodyId)->getCollisionShape()->calculateLocalInertia(mass,fallInertia);
					//cerr<<"setting mass:"<<mass<<endl;
				}

								
				if( !(bodyIt->second.bodyId)->isStaticObject() && (bodyIt->second.bodyId)->isActive() == 1 ) // if not tagged as Static
				{
					//Velocity
					btVector3 v;
					v = (bodyIt->second.bodyId)->getLinearVelocity(); 
					rbdstate->setVelocity( UT_Vector3( v.getX(), v.getY(), v.getZ() ) );
					//Angular Velocity
					btVector3 w;
					w = (bodyIt->second.bodyId)->getAngularVelocity(); 
					rbdstate->setAngularVelocity( UT_Vector3( w.getX(), w.getY(), w.getZ() ) );
				}
			}	
		}
	}

	return SIM_Solver::SIM_SOLVER_SUCCESS;
}

//Adds a Bullet body to the world
//Based on type of geometry in the SIM_Object we make a Bullet body to match
std::map< int, bulletBody >::iterator SIM_SnowSolverBullet::addBulletBody(SIM_Object *currObject)
{
	std::map< int, bulletBody >::iterator bodyIt;
	bodyIt = state->m_bulletBodies->end();
	SIM_Geometry *myGeo;
	const SIM_SDF *sdf;
	UT_Vector3 p, v, w;
	UT_Quaternion rot;

	//cout<<"adding simulation object to bullet :"<<currObject->getName()<<endl;

	// Get the Geometry...	
	myGeo = (SIM_Geometry *)currObject->getGeometry();
	
	//We have geomoetry
	if(myGeo)
	{
		//Find out the type
		sdf = SIM_DATA_GETCONST(*myGeo, SIM_SDF_DATANAME, SIM_SDF);
		bulletBody	currBody;
		
		RBD_State *rbdstate = SIM_DATA_GET(*currObject, "Position", RBD_State);
		SIM_SnowBulletData *bulletstate = SIM_DATA_GET(*currObject, "Bullet Snow Data", SIM_SnowBulletData);
		if(rbdstate) //This is an rbd object
		{
			UT_DMatrix4 xform;
			SIMgetGeometryTransform(xform, *currObject);
			// TODO: support this xform. Currently I don't know how to get the CompoundShape, COM and such to work.
			
			// Retrieve the gdp...
			GU_DetailHandleAutoReadLock	gdl(myGeo->getGeometry());
			GU_Detail *gdp = (GU_Detail *)gdl.getGdp();	
			btCollisionShape* fallShape;
			fallShape = NULL;
			UT_BoundingBox bbox;
						
			p = rbdstate->getPosition();
			rot = rbdstate->getOrientation();

			// default motion state, will be set by sim loop later
			btDefaultMotionState* fallMotionState = new btDefaultMotionState( 
					btTransform( btQuaternion(rot.x(),rot.y(),rot.z(),rot.w()),btVector3(p[0],p[1],p[2]) ) );
			
			UT_String geoRep;
			geoRep = GEO_REP_AS_IS;
			if( bulletstate )
			{
				bulletstate->getGeoRep(geoRep);
			}
			
			if( geoRep == GEO_REP_AS_IS ) // geometry as-is
			{
				GEO_Primitive *prim;
				GEO_Point *pt;
				GEO_PrimSphere *sphere;
				int ntriangles=0, nspheres=0;
				FOR_ALL_PRIMITIVES (gdp, prim) 
				{
					if (prim->getPrimitiveId() & GEOPRIMPOLY)
						if( prim->getVertexCount() == 3 )  //add only tri's
							ntriangles++;
					if (prim->getPrimitiveId() & GEOPRIMSPHERE)
						nspheres++;
				}
				if( ntriangles )
				{
					//cout<<"creating triangle collision shape"<<endl;
					// Transfer point positions
					btVector3 *gVertices = new btVector3[gdp->points().entries()];
					int numVertices = 0;
					int vertStride = sizeof(btVector3);
					FOR_ALL_GPOINTS( gdp, pt )
					{
						UT_Vector3 ppt = pt->getPos(); // * xform;
						gVertices[numVertices++] = btVector3( 
							ppt.x(), ppt.y(), ppt.z() );
					}
					// Transfer Indices
					int *gIndices = new int[gdp->primitives().entries()*3];
					int indexStride = 3*sizeof(int);
					int numIndices = 0;
					FOR_ALL_PRIMITIVES (gdp, prim)
						for( int i=0; i<3; i++ )
							gIndices[numIndices++] = prim->getVertex(2-i).getPt()->getNum();
	
					btTriangleIndexVertexArray* indexVertexArray = new btTriangleIndexVertexArray(
						ntriangles,
						gIndices,	indexStride,
						numVertices, (btScalar*) &gVertices[0].x(), vertStride);

					bool doConvex=false;
					if( bulletstate )
						doConvex = bulletstate->getGeoConvex();
						
					if( doConvex )
					{
						//convex-hull collision shape
						fallShape = new btConvexTriangleMeshShape(indexVertexArray); 
					}
					else
					{					
						// concave collision shape
						btGImpactMeshShape *impactShape = new btGImpactMeshShape(indexVertexArray); 
						impactShape->updateBound();
						fallShape = impactShape;
						btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>
								(state->m_dynamicsWorld ->getDispatcher());
						btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
					}
				}
				else if ( nspheres == 1 )
				{
					//cout<<"creating single sphere shape"<<endl;
					
					// FIXED BY SRH 2010-03-30 //
					FOR_ALL_PRIMITIVES (gdp, prim) 
					{
						sphere = dynamic_cast<GEO_PrimSphere*>( prim );
						
						// Set the sphere's scale (based off of both its radius setting and any scale transforms)
						UT_Matrix4 xform;
						sphere->getTransform4( xform );
						UT_XformOrder xformOrder;
						UT_Vector3 rotation, scale, translation;
						xform.explode( xformOrder, rotation, scale, translation );
						float sphereRadius = scale.x();
						fallShape = new btSphereShape( sphereRadius );
					}
					// *********************** //
				}
				else if ( nspheres > 1 )
				{
				    // MAKE MULTIPLE SPHERES AND CONSTRAIN THEM
				    //   Uses srh's shCompoundSphereShape added to Bullet
				    //cout<<"creating multisphere shape"<<endl;
				    
				    // ADDED BY SRH 2010-02-22 //
				    btAlignedObjectArray<btSphereShape*> sphereShapes;
				    btAlignedObjectArray<btVector3> sphereRelativePositions;
				    // *********************** //
					
					btScalar *sphereRadii = new btScalar [nspheres];
					btVector3 *sphereCentres = new btVector3 [nspheres];
					//btVector3 centerOfMass( 0.0, 0.0, 0.0 );
					int c = 0;
					UT_Vector3 centre;
					FOR_ALL_PRIMITIVES (gdp, prim) 
					{
						if (prim->getPrimitiveId() & GEOPRIMSPHERE)
						{
							sphere = dynamic_cast<GEO_PrimSphere*>( prim );
							
							// Sphere radius (based off of both its radius setting and any scale transforms)
							// ADDED BY SRH 2010-02-22 //
							UT_Matrix4 xform;
							sphere->getTransform4( xform );
							UT_XformOrder xformOrder;
							UT_Vector3 rotation, scale, translation;
							xform.explode( xformOrder, rotation, scale, translation );
							sphereRadii[c] = scale.x();		// THIS NEEDS FIXED TO TAKE THE LARGEST (OR SMALLEST) DIMENSION (not just x)
							// *********************** //
							
							// Sphere center
							centre = sphere->baryCenter();
							sphereCentres[c] = btVector3( centre.x(), centre.y(), centre.z() );
							//centerOfMass = centerOfMass + sphereCentres[c];
							//cout << " Pos = " << centre.x() << ", " << centre.y() << ", " << centre.z() << endl;
							
							// ADDED BY SRH 2010-02-22 //
							sphereShapes.push_back( new btSphereShape(sphereRadii[c]) );
							sphereRelativePositions.push_back( sphereCentres[c] );
							// *********************** //
							
							c++;
						}
						
					}  // FOR_ALL_PRIMITIVES
					
					// ADDED BY SRH 2010-02-22 //
					fallShape = new shCompoundSphereShape( sphereShapes, sphereRelativePositions );
					// *********************** //
					
					delete sphereRadii;
					delete sphereCentres;
				}
				else if( ntriangles == 0  && nspheres == 0 )
				{
					cout<<"Bullet Solver: Warning! There are no spheres nor triangles in the simulation object:"<<currObject->getName()<<endl;
				}
			}
			else if( geoRep == GEO_REP_SPHERE )  // sphere representation
			{
				gdp->getBBox(&bbox);
				if( bulletstate->getAutofit() )
					fallShape = new btSphereShape( bbox.maxvec().length() );
				else	
					fallShape = new btSphereShape( abs(bulletstate->getPrimRadius()) );
			}
			else if( geoRep == GEO_REP_BOX )  // box representation
			{
				gdp->getBBox(&bbox);
				if( bulletstate->getAutofit() )
					fallShape = new btBoxShape( btVector3( bbox.xsize(),bbox.ysize(),bbox.zsize() ) );
				else
				{
					UT_Vector3 prim_s = bulletstate->getPrimS();
					fallShape = new btBoxShape( btVector3(prim_s.x(),prim_s.y(),prim_s.z()) );
				}
			}
			else if( geoRep == GEO_REP_CAPSULE )  // capsule representation
			{
				fallShape = new btCapsuleShape( abs(bulletstate->getPrimRadius()), 
					abs(bulletstate->getPrimLength()) );
			}

			// now add the shapes to bullet
			if( fallShape )
			{
				// Physical Properties
				btScalar mass = 1.0f;
				btScalar restitution = 0.5;
				btScalar friction = 0.2;
				//btScalar linear_damping = 0;
				//btScalar angular_damping = 0;	
				SIM_PhysicalParms *physicalparms = SIM_DATA_GET(*currObject, 
					"PhysicalParms", SIM_PhysicalParms);
				if( physicalparms )
				{
					restitution = physicalparms->getBounce();
					friction = physicalparms->getFriction();
				}
	
				// Calculate mass and inertia
				if( currObject->getIsStatic() )
					mass = 0;
				else
					mass = rbdstate->getMass();
				btVector3 fallInertia(0,0,0);
				fallShape->calculateLocalInertia(mass,fallInertia);
				
				// collision margin / tolerance
				float collisionMargin = 0.0f;
				if( bulletstate )
					collisionMargin = bulletstate->getCollisionMargin();
				fallShape->setMargin( collisionMargin ); 

				// convert to btCompoundShape in order to offset the COM. 
				// This behaves strangely right now though.. at least, to me.  Just storing these lines for later..
				//btCompoundShape* compoundShape = new btCompoundShape();
				//UT_Vector3 xform_t;
				//xform.getTranslates(xform_t);
				//btTransform btCom( btRot, btP );
				//btCom *= convertTobtTransform(xform);
				//btCom.setIdentity();
				//btCom.setOrigin( btVector3( xform_t.x(),xform_t.y(),xform_t.z() ) );				
				//compoundShape->addChildShape( btCom, fallShape );
				//compoundShape->addChildShape( convertTobtTransform(xform), fallShape );
				//compoundShape->calculateLocalInertia(mass,fallInertia);
				
				btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(
					mass,
					fallMotionState,
					//compoundShape,
					fallShape,
					fallInertia);
				fallRigidBodyCI.m_friction = friction;
				fallRigidBodyCI.m_restitution = restitution;
	
				// Initialize a new body
				btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
				state->m_dynamicsWorld->addRigidBody(fallRigidBody);
				//cout<<"creating new body, id:"<<currObject->getObjectId()
				//	<<"  isStaticObject:"<<fallRigidBody->isStaticObject()<<endl;
	
				// Insert the body into the map
				currBody.bodyId	= fallRigidBody;
				currBody.isStatic =	1;
				currBody.dopId = currObject->getObjectId();
				currBody.type = "body";
				state->m_bulletBodies->insert(std::make_pair( currObject->getObjectId() , currBody	));	
				bodyIt = state->m_bulletBodies->find( currObject->getObjectId() );
			}
		}	
	}
	
	//Return the body we found
	return bodyIt;

}



// ADDED BY SRH 2010-03-31, THIS CODE TAKEN FROM SIM_SolverODE.C *************** //
std::map< int, bulletBody >::iterator SIM_SnowSolverBullet::addAffector( SIM_Object *currObject )
{
    //SIM_ODEWorldData *worlddata;
    //worlddata = SIM_DATA_CREATE(*this, "ODE_World", SIM_ODEWorldData, SIM_DATA_RETURN_EXISTING);
    //ObjectMap *odeaffectors = worlddata->getOdeAffectors();

	std::map< int, bulletBody >::iterator bodyIt;
	const SIM_Geometry *simgeo;
	const SIM_SDF *sdf;
	//const SIM_EmptyData *bulletInfo;
	RBD_State *rbdState;
	bulletBody currBody;

	simgeo = currObject->getGeometry();

	if (simgeo)
	{
		// We have geometry to work with.
		
		// Check if we have an sdf
		sdf = SIM_DATA_GETCONST(*simgeo, SIM_SDF_DATANAME, SIM_SDF);
		
		/*if (sdf)
		{
		    if (sdf->getMode() == 4)	// This is a ground plane
		    {
				if ( !createGroundPlane(currObject, currBody) )
				{
				    return bodyIt;
				}
				
				//Insert the body into the map
				m_bulletAffectors->insert( std::make_pair(currObject->getObjectId(), currBody) );
				bodyIt = m_bulletAffectors->find( currObject->getObjectId() );
				// Store the OdeObject data for later reference
				dGeomSetData( currBody.geomId, &bodyIt->second );
		
				return bodyIt;
		    }
		}*/
		
		//RBD_State *rbdstate = SIM_DATA_GET(*currObject, "Position", RBD_State);
		SIM_SnowBulletData *bulletstate = SIM_DATA_GET(*currObject, "Bullet Snow Data", SIM_SnowBulletData);
		//odeinfo = SIM_DATA_GETCONST( *currObject, "ODE_Body", SIM_EmptyData);
		/*if (!bulletstate)
		{
		    cerr << "No Bullet info found!" << endl;
		    return bodyIt;
		}*/
	
		/*
		// Check if this is a composite object, and handle it if so
		OdePrimType primtype = 
			(OdePrimType)odeinfo->getData().getOptionI("primType");
		if (primtype == odeComposite)
		{
		    // The relevant OdeObject data needs to already be in the 
		    // affector map before we call createComposite, because we 
		    // essentially need a pointer to the OdeObject to give to every 
		    // geom that's created as part of the composite.
		    odeaffectors->insert(std::make_pair(
				    currobject->getObjectId(), currbody));
		    bodyitr = odeaffectors->find(currobject->getObjectId());
		    if (!createComposite(currobject, bodyitr, true))
		    {
			// If we weren't successful, remove the OdeObject
			odeaffectors->erase(bodyitr);
			bodyitr = odeaffectors->end();
		    }
		    else
		    {
			bodyitr->second.isStatic = false;
			bodyitr->second.isAffector = true;
		    }
		    return bodyitr;
		}
		*/
	
		// Attempt to get its State
		//state = SIM_DATA_GET(*currobject, "Position", RBD_State);
		RBD_State *rbdstate = SIM_DATA_GET(*currObject, "Position", RBD_State);
		
		/*if (rbdstate) // This is an RBD object
		{
		    if ( !createBody(currObject, currBody, true) )
		    {
				return bodyIt;
		    }
		}
		else	// This is a static object
		{
		    if ( !createStatic(currObject, currBody) )
		    {
				return bodyitr;
		    }
		}*/
	
		// Insert the new affector into the map
		state->m_bulletAffectors->insert(std::make_pair( currObject->getObjectId(), currBody) );
		bodyIt = state->m_bulletAffectors->find( currObject->getObjectId() );
	
		// Store the OdeObject data for later reference
		//dGeomSetData( currBody.geomId, &bodyIt->second);
    }

    return bodyIt;
    
}  // addAffectors()
// ***************************************************************************** //




//Kill off the bullet bodies that are not needed anymore
void SIM_SnowSolverBullet::removeDeadBodies(SIM_Engine &engine)
{
	std::map< int, bulletBody >::iterator bodyIt;
	std::vector<int> deadBodies;
	
	// Accumulate all bodies to remove
	for( bodyIt = state->m_bulletBodies->begin(); bodyIt != state->m_bulletBodies->end(); ++bodyIt )
	{
		SIM_Object *currObject = (SIM_Object *)engine.getSimulationObjectFromId(bodyIt->first);
		//This object no longer exists, destroy the bullet body (add to a map)
		if(!currObject)
		{
			//cout<<"removing bullet body, id:"<<currObject->getObjectId()<<endl;
			deadBodies.push_back(bodyIt->first);
		}
	}

	// ... remove them from bullet
	for(int b = 0; b < deadBodies.size(); b++)
	{
		bodyIt	= state->m_bulletBodies->find( deadBodies[b] );
		state->m_dynamicsWorld->removeRigidBody( (bodyIt->second.bodyId) );
		delete bodyIt->second.bodyId;
		//Erase map entry
		state->m_bulletBodies->erase(bodyIt);
	}
}


//Applies forces and constraints
//This loop could include more stuff since we're 
//looping over all subData.. so may have to make it more
//general
void SIM_SnowSolverBullet::processSubData(SIM_Object *currObject, bulletBody &body)
{	
	int f;
	UT_String forcename;
	const SIM_Data *dopData;
	
	body.bodyId->clearForces();
	
	//Loop over subdata and get forces we're interested in
	for(f = 0; f< currObject->getNumSubData(); f++)
	{
			
		//Apply Uniform Force
		if(dopData = currObject->getNthConstSubData( 
			&forcename, SIM_DataFilterByType("SIM_ForceUniform"), f, 0, SIM_DataFilterAll() ))
		{
			const SIM_ForceUniform *dopforce = SIM_DATA_CASTCONST( dopData, SIM_ForceUniform );
			UT_Vector3 thisforce = dopforce->getUniformForce();
			UT_Vector3 thistorque = dopforce->getUniformTorque();
			body.bodyId->applyCentralForce( btVector3(thisforce[0],thisforce[1],thisforce[2]) );
			//cout<<"adding central (uniform) force to id:"<<currObject->getObjectId()<<endl;
		}

		//Apply Gravity Force
		if(dopData = currObject->getNthConstSubData( 
			&forcename, SIM_DataFilterByType("SIM_ForceGravity"), f, 0, SIM_DataFilterAll() ))
		{
			const SIM_ForceGravity *dopforce = SIM_DATA_CASTCONST( dopData, SIM_ForceGravity );
			UT_Vector3 thisforce = dopforce->getGravity();
			body.bodyId->setGravity(btVector3(thisforce[0],thisforce[1],thisforce[2]));
			//cout<<"adding gravity force"<<endl;
		}

		//Apply Point Force
		if(dopData = currObject->getNthConstSubData( 
			&forcename, SIM_DataFilterByType("SIM_ForcePoint"), f, 0, SIM_DataFilterAll() ))
		{
			const SIM_ForcePoint *dopforce = SIM_DATA_CASTCONST( dopData, SIM_ForcePoint );
			UT_Vector3 thisforce = dopforce->getForce();
			UT_Vector3 thisPos = dopforce->getForcePosition();
			body.bodyId->applyForce( btVector3(thisforce[0],thisforce[1],thisforce[2]),
				btVector3(thisPos[0],thisPos[1],thisPos[2]) );
			//cout<<"adding (point) force to id:"<<currObject->getObjectId()<<endl;
		}

	}
}



void SIM_SnowSolverBulletState::initSystem(  )
{

	//cout<<"creating world: start"<<endl;
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher( m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
#ifdef DO_SNOW_STUFF
	m_solver = new shMolecularDynamicsConstraintSolver();
#else
    m_solver = new btSequentialImpulseConstraintSolver();
#endif
	
#ifdef DO_SNOW_STUFF
    m_dynamicsWorld = new shGranularDiscreteDynamicsWorld( 
		m_dispatcher,
		m_broadphase,
		m_solver,
		m_collisionConfiguration);
	//m_dynamicsWorld = world;
#else
    m_dynamicsWorld = new btDiscreteDynamicsWorld( 
		m_dispatcher,
		m_broadphase,
		m_solver,
		m_collisionConfiguration);
#endif
	
    m_dynamicsWorld->clearForces();

	m_bulletBodies = new std::map<int, bulletBody>();
	m_bulletAffectors = new std::map<int, bulletBody>();


	// TEMP: add groundplane
	/*
	cout<<"creating temp groundplane"<<endl;
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
	btDefaultMotionState* groundMotionState = new btDefaultMotionState( 
		btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	m_dynamicsWorld->addRigidBody(groundRigidBody);
	*/
}

void SIM_SnowSolverBulletState::cleanSystem(  )
{
	
	if(  m_dynamicsWorld != NULL )
	{	
		if(  refCount == 0 )
		{	
			//cout<<"destroying world: start "<< m_dynamicsWorld<<endl;
			// delete in reverse order than creation
			for( int i =  m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i ) {
				btCollisionObject* obj =  m_dynamicsWorld->getCollisionObjectArray()[i];
				btRigidBody* body = btRigidBody::upcast( obj );
				if( body && body->getMotionState() )
				delete body->getMotionState();		
				m_dynamicsWorld->removeCollisionObject( obj );
				delete obj;
			}
		
			delete  m_dynamicsWorld;
			delete  m_solver;
			delete  m_broadphase;
			delete  m_dispatcher;
			delete  m_collisionConfiguration;
			m_collisionConfiguration = NULL;
			m_dispatcher = NULL;
			m_broadphase = NULL;
			m_solver = NULL;
			m_dynamicsWorld = NULL;
			m_bulletBodies->clear();
			delete  m_bulletBodies;
			//cout<<"destroying world: end"<<endl;
		}
	}

}



void initializeSIM(void *)
{
	//register our stuff with houdini
	//
	IMPLEMENT_DATAFACTORY(SIM_SnowSolverBullet);
	IMPLEMENT_DATAFACTORY(SIM_SnowBulletData);
}


// ---------------------


const SIM_DopDescription*
SIM_SnowBulletData::getSnowBulletDataDopDescription()
{
	static PRM_Name                theGeoRep(SIM_NAME_GEO_REP, "Geometry Representation");
	static PRM_Name                theGeoTri(SIM_NAME_GEO_TRI, "Triangulate Polygons (not working yet)");
	static PRM_Name                theGeoConvex(SIM_NAME_GEO_CONVEX, "Polygons As Convex Hulls");
	static PRM_Name                thePrimT(SIM_NAME_PRIM_T, "Prim Translate");
	static PRM_Name                thePrimR(SIM_NAME_PRIM_R, "Prim Rotate");
	static PRM_Name                thePrimS(SIM_NAME_PRIM_S, "Prim Scale");
	static PRM_Name                thePrimRadius(SIM_NAME_PRIM_RADIUS, "Prim Sphere Radius");
	static PRM_Name                thePrimLength(SIM_NAME_PRIM_LENGTH, "Prim Capsule Length");
	static PRM_Name                thePrimAutofit(SIM_NAME_PRIM_AUTOFIT, "AutoFit Primitive Spheres or Boxes to Geometry");
	static PRM_Name                theCollisionMargin(SIM_NAME_COLLISION_MARGIN, "Collision Tolerance");

	static PRM_Name		theGeoRepNames[] = {
		PRM_Name(GEO_REP_AS_IS,		"As Is"),
		PRM_Name(GEO_REP_SPHERE,	"Sphere"),
		PRM_Name(GEO_REP_BOX,		"Box"),
		PRM_Name(GEO_REP_CAPSULE,	"Capsule"),
		PRM_Name(0)
	};
	static PRM_ChoiceList   theGeoRepNamesMenu((PRM_ChoiceListType)(PRM_CHOICELIST_REPLACE | PRM_CHOICELIST_EXCLUSIVE ), &(theGeoRepNames[0]) );

	static PRM_Default             defZero(0);
	static PRM_Default             defOne(1);
	static PRM_Default             defGeoRep(0, GEO_REP_AS_IS);
	static PRM_Default             defCollisionMargin(0.0);
	static PRM_Default defZeroVec[]= {PRM_Default(0), PRM_Default(0), PRM_Default(0)};
	static PRM_Default defOneVec[] = {PRM_Default(1), PRM_Default(1), PRM_Default(1)};

	static PRM_Range               geoRepRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 5);
	static PRM_Range               geoGeoTriRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 1);
	static PRM_Range               primRadiusRange(PRM_RANGE_UI, 0.1, PRM_RANGE_UI, 5);
	static PRM_Range               primLengthRange(PRM_RANGE_UI, 0.1, PRM_RANGE_UI, 5);
	static PRM_Range               collisionMarginRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 0.5);
	
	static PRM_Template            theTemplates[] = {
		PRM_Template(PRM_STRING,		1, &theGeoRep, &defGeoRep, &theGeoRepNamesMenu),
		PRM_Template(PRM_TOGGLE_J,	1, &theGeoTri, &defOne),
		PRM_Template(PRM_TOGGLE_J,	1, &theGeoConvex, &defZero),
		PRM_Template(PRM_XYZ,		3, &thePrimT, defZeroVec),
		PRM_Template(PRM_XYZ,		3, &thePrimR, defZeroVec),
		PRM_Template(PRM_XYZ,		3, &thePrimS, defOneVec),
		PRM_Template(PRM_FLT_J,		1, &thePrimRadius, &defOne, 0, &primRadiusRange),
		PRM_Template(PRM_FLT_J,		1, &thePrimLength, &defOne, 0, &primLengthRange),
		PRM_Template(PRM_TOGGLE_J,	1, &thePrimAutofit, &defZero, 0, &geoGeoTriRange),
		PRM_Template(PRM_FLT_J,		1, &theCollisionMargin, &defCollisionMargin, 0, &collisionMarginRange),
		PRM_Template()
	};
	
	static SIM_DopDescription    theDopDescription(true,
							"snowbulletdata",
							getName(),
							"Bullet Snow Data",
							classname(),
							theTemplates);
	
	return &theDopDescription;
}

const char* SIM_SnowBulletData::getName() 
{
	static char name[256];
	sprintf (name, "Bullet Snow Data");
	return name;
}

SIM_SnowBulletData::SIM_SnowBulletData(const SIM_DataFactory *factory)
: BaseClass(factory), SIM_OptionsUser(this)
{
}

SIM_SnowBulletData::~SIM_SnowBulletData()
{
}

void
SIM_SnowSolverBulletState::addReference()
{
	++refCount;
}

void
SIM_SnowSolverBulletState::removeReference()
{
	--refCount;
	if (refCount < 1)
		delete this;
}

SIM_SnowSolverBulletState::SIM_SnowSolverBulletState() : refCount(0)
{
	initSystem();
}

SIM_SnowSolverBulletState::~SIM_SnowSolverBulletState()
{
	if( m_dynamicsWorld != NULL )
		cleanSystem();
}


