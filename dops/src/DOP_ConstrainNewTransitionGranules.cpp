#include "DOP_ConstrainNewTransitionGranules.h"
#include <UT/UT_DSOVersion.h>

#include <RBD/RBD_State.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_EmptyData.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_Options.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipGroup.h>
#include <SIM/SIM_GlueNetworkRelationship.h>
#include <SIM/SIM_RelationshipCollide.h>
#include <SIMZ/SIM_SopGeometry.h>
#include <OP/OP_OperatorTable.h>
#include <DOP/DOP_PRMShared.h>
#include <DOP/DOP_InOutInfo.h>
#include <DOP/DOP_Engine.h>


#include <iostream>



void
newDopOperator(OP_OperatorTable *table)
{
    OP_Operator *op;

    // Create a new DOP_Operator which describes the operator we are
    // building. The parameters to this function are similar to the
    // OP_Operator constructor except for the last parameter, which
    // specifies the number of outputs (up to 4) from this operator.
    op = new DOP_ConstrainNewTransitionGranulesOperator("hdk_constrainnewtransitiongranules", "Constrain Transition Granules",
                          DOP_ConstrainNewTransitionGranules::myConstructor,
                          DOP_ConstrainNewTransitionGranules::myTemplateList, 1, 9999, 0,
                          0, 1);
    table->addOperator(op);
}




#define     POINT_CLASS     0
#define     PRIM_CLASS      1
#define     DETAIL_CLASS    2
#define     INT_TYPE        0
#define     FLOAT_TYPE      1
#define     STRING_TYPE     2

static PRM_Name         theConnectedGroupsName( "connectedgroupsname", "Connected Groups Prefix" );
static PRM_Name         theConstraintObjectsName( "constraintobjectsname", "Constraint Objs Prefix" );
static PRM_Name         theTransitionObjectsName( "transitionobjectsname", "Transition Objs Prefix" );
static PRM_Name         theGranuleObjectsName( "granuleobjectsprefix", "Granule Objs Prefix" );
static PRM_Name         theCollideRelName( "colliderelname", "Collide Rel Name" );
static PRM_Name         theSolveFirstFrame( "solvefirstframe", "Solve On Creation Frame" );
static PRM_Name         theSolidMeshGeomData( "solidmeshgeomdata", "Solid Mesh Data" );
static PRM_Name         theInteriorPointsGeomData( "interiorpointsgeomdata", "Interior Geom Data" );
static PRM_Name         theCullMetasGeomData( "cullmetasgeomdata", "Cull Metas Mesh Data" );
static PRM_Name         theCullVolumeGeomData( "cullvolumegeomdata", "Cull Volume Geom Data" );
static PRM_Name         theGranuleData( "granuledata", "Granule Data" );

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0 );

PRM_Template
DOP_ConstrainNewTransitionGranules::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template( PRM_INT_J,    1, &DOPactivationName, &DOPactivationDefault ),
    
    // My added parameters
    PRM_Template( PRM_STRING,   1, &theConnectedGroupsName, &defaultNull, 0, 0, 0, 0, 1, "Name prefix of the groups that hold all the interior granules grouped with their neighbors." ),
    PRM_Template( PRM_STRING,   1, &theConstraintObjectsName, &defaultNull, 0, 0, 0, 0, 1, "Name prefix of the existing objects that transition granules are constrained to." ),
	PRM_Template( PRM_STRING,   1, &theTransitionObjectsName, &defaultNull, 0, 0, 0, 0, 1, "Name prefix of the transition granule objects." ),
	PRM_Template( PRM_STRING,   1, &theGranuleObjectsName, &defaultNull, 0, 0, 0, 0, 1, "Name prefix of all the granule objects (no matter what granuleType label they have)." ),
	PRM_Template( PRM_STRING,   1, &theCollideRelName, &defaultNull, 0, 0, 0, 0, 1, "Name of the collide relationship to add new solid meshes to." ),
	PRM_Template( PRM_TOGGLE_J, 1, &theSolveFirstFrame, &defaultNull, 0, 0, 0, 0, 1, "For the newly created objects, this parameter controls whether or not the solver for that object should solve for the object on the timestep in which it was created. Usually this parameter will be turned on if this node is creating objects in the middle of a simulation rather than creating objects for the initial state of the simulation." ),
	//PRM_Template( PRM_TOGGLE_J,  1, &DOPsolvefirstframeName, &defaultZero, 0, 0, 0, 0, 1, "For the newly created objects, this parameter controls whether or not the solver for that object should solve for the object on the timestep in which it was created. Usually this parameter will be turned on if this node is creating objects in the middle of a simulation rather than creating objects for the initial state of the simulation." ),
	PRM_Template( PRM_STRING,   1, &theSolidMeshGeomData, &defaultNull, 0, 0, 0, 0, 1, "Name of the Geometry data in the constraint object that contains the surface mesh geometry of the solid granular mesh." ),
	PRM_Template( PRM_STRING,   1, &theInteriorPointsGeomData, &defaultNull, 0, 0, 0, 0, 1, "Name of the Geometry data in the constraint object that contains the points representing the most recently deleted interior granules." ),
    PRM_Template( PRM_STRING,   1, &theCullMetasGeomData, &defaultNull, 0, 0, 0, 0, 1, "Name of the Geometry data in the constraint object that contains the impact culling metaball geometry ." ),
	PRM_Template( PRM_STRING,   1, &theCullVolumeGeomData, &defaultNull, 0, 0, 0, 0, 1, "Name of the Geometry data in the constraint object that contains the volume geometry of the carved away area of the solid mesh, if any." ),
	PRM_Template( PRM_STRING,   1, &theGranuleData, &defaultNull, 0, 0, 0, 0, 1, "Name of the Granule data that keeps track of info such as whether or not its object is a granule." ),
	PRM_Template()
};

OP_Node *
DOP_ConstrainNewTransitionGranules::myConstructor(OP_Network *net, const char *name,
                                 OP_Operator *op)
{
    return new DOP_ConstrainNewTransitionGranules(net, name, op);
}

DOP_ConstrainNewTransitionGranules::DOP_ConstrainNewTransitionGranules(OP_Network *net, const char *name,
                                     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_ConstrainNewTransitionGranules::~DOP_ConstrainNewTransitionGranules()
{
}

void
DOP_ConstrainNewTransitionGranules::processObjectsSubclass(fpreal time, int forOutputIdx,
                                          const SIM_ObjectArray &objects,
                                          DOP_Engine &engine)
{
	// Get the connected groups (connectedGroups)
	UT_String connectedGroupsPrefix;
	CONNECTEDGROUPSPREFIX( connectedGroupsPrefix, time );
    connectedGroupsPrefix.strip("*");
	if ( connectedGroupsPrefix == "" )
		return;
	SIM_ConstDataArray connectedGroups;
	UT_String connectedGroupFilterName = connectedGroupsPrefix;		// groupMask is the group prefix name (e.g. neighborsOf*)
	connectedGroupFilterName += "*";
	SIM_DataFilterByName connectedGroupFilter( connectedGroupFilterName );
	engine.filterConstRelationships( connectedGroupFilter, connectedGroups );
	
	
	// Get the constraint objects that new transition granules should be constrained to (constraintObjects)
	UT_String constraintObjectsPrefix;
	CONSTRAINTOBJECTSPREFIX( constraintObjectsPrefix, time );
	constraintObjectsPrefix.strip("*");
	if ( constraintObjectsPrefix == "" )
		return;
	UT_String constraintObjectsFilterName = constraintObjectsPrefix;
	constraintObjectsFilterName += "*";
    SIM_DataFilterByName constraintObjectsFilter( constraintObjectsFilterName );
	
	
	// Get the prefix of the names of all the transition granule objects
	UT_String transitionGranuleObjectsPrefix;
	GRANULEOBJECTSPREFIX( transitionGranuleObjectsPrefix, time );
	transitionGranuleObjectsPrefix.strip("*");
	if ( transitionGranuleObjectsPrefix == "" )
		return;
	UT_String transitionObjectsFilterName = transitionGranuleObjectsPrefix;
	transitionObjectsFilterName += "*";
	SIM_DataFilterByName transitionObjectsFilter( transitionObjectsFilterName );
	
	
	// Get the prefix of the names of all the granule objects
	UT_String granuleObjectsPrefix;
	GRANULEOBJECTSPREFIX( granuleObjectsPrefix, time );
	
	
	// Get the name of the collide relationship to add new solid meshes (CONSTR) to
	UT_String collideRelName;
	COLLIDERELNAME( collideRelName, time );
	
	
	// Get whether or not to solve on the creation frame
	int doSolveOnCreationFrame = SIMULATECREATIONFRAME( time );
	
	
	// Get the name of the Geometry data that contains the solid mesh geometry
	UT_String solidMeshGeomDataName;
	SOLIDMESHGEOMETRYDATA( solidMeshGeomDataName, time );
	
	// Get the name of the Geometry data that contains the solid mesh geometry
	UT_String interiorGranulePointsGeomDataName;
	INTERIORGRANULEPOINTSGEOMETRYDATA( interiorGranulePointsGeomDataName, time );
	
	// Get the name of the Geometry data that contains the cull metas from impacts
	UT_String cullMetasGeomDataName;
	CULLMETASGEOMETRYDATA( cullMetasGeomDataName, time );
	
	// Get the name of the Geometry data that contains the culled volume from impacts
	UT_String cullVolumeGeomDataName;
	CULLVOLUMEGEOMETRYDATA( cullVolumeGeomDataName, time );
	
	// Get the name of the Granule data
	UT_String granuleDataName;
	GRANULEDATA( granuleDataName, time );
	
	
	
	if( !isActive(time) )
	{
		return;
	}  // if
	
	
	
	// Set the transition granule constraints ... constrain them to their respective CONSTR objects
	/*
	SIM_ObjectArray transObjs;
	objects.filter( transitionObjectsFilter, transObjs );
	int numTransObjs = transObjs.entries();
	for ( int i = 0; i < numTransObjs; i++ )
	{
		// Get the object
		SIM_Object* curTransObj = transObjs(i);
		
		// Get the granule type
		SIM_EmptyData* data = SIM_DATA_GET( *curTransObj, "GranuleData", SIM_EmptyData );
		SIM_Options& options = data->getData();
		UT_String granuleType;
		options.getOptionString( "granuleType", granuleType );
		
		if ( granuleType == "transition" )
		{
			// Get the glue object (solid mesh) from the position data
			RBD_State *rbdstate = SIM_DATA_GET( *curTransObj, "Position", RBD_State );
			UT_String glueObjectName;
			rbdstate->getGlueObject( glueObjectName );
			if ( glueObjectName == "" )		// The transition granule could have just broken off (from a collision) and thus will no longer have an associated solid mesh
			{
				continue;
			}  // if
			SIM_Object* curGlueObj = (SIM_Object*)engine.findObjectFromString( glueObjectName, 0, 0, time, 0 );
			
			// Get the glue relationship that the glue object (solid mesh) belongs to 
			SIM_DataFilterByName glueRelFilter( "glueConstraint*" );
			SIM_ConstDataArray glueRels;
			curGlueObj->filterConstRelationships( true, glueRelFilter, glueRels );
			SIM_Relationship* glueRel;
			
			// If the glue object (solid mesh) is not a part of any "glueConstraint" relationship, then create and add it to a new one
			if ( glueRels.entries() == 0 )
			{
				char tmp2[181]; sprintf( tmp2, "%s%d", "glueConstraint", curGlueObj->getObjectId() );
				UT_String glueName = tmp2;
				glueRel = engine.addRelationship( glueName, SIM_DATA_RETURN_EXISTING );
				glueRel->addGroup( curGlueObj );
				glueRel->addAffGroup( curGlueObj );      // currObject is the interior granule, since it comes from interiorGranulesFiltered
				SIM_DATA_CREATE( *glueRel, "Group",
								SIM_RelationshipGroup,
								SIM_DATA_RETURN_EXISTING );
			}  // if
			else
			{
				glueRel = (SIM_Relationship*)glueRels(0);
			}  // else
			
			// Glue the current transition granules to its corresponding glue object (solid mesh)
			//   by adding it to the solid mesh's "glueConstraint" group
			glueRel->addGroup( curTransObj );
			glueRel->addAffGroup( curTransObj );
			SIM_DATA_CREATE( *glueRel, "Group",
							SIM_RelationshipGroup,
							SIM_DATA_RETURN_EXISTING );
		}  // if
	}  // for i
	*/
	
	
	
	
	
	// For each CONSTR, clear out the interior granules point geometry data.
	SIM_ObjectArray constrObjs;
	objects.filter( constraintObjectsFilter, constrObjs );
	int numConstrObjs = constrObjs.entries();
	for ( int i = 0; i < numConstrObjs; i++ )
	{
		SIM_Object* curConstr = constrObjs(i);
		SIM_GeometryCopy* intPtsGeom = SIM_DATA_GET( *curConstr, interiorGranulePointsGeomDataName, SIM_GeometryCopy);
		if ( !intPtsGeom )
			intPtsGeom = SIM_DATA_CREATE( *curConstr, interiorGranulePointsGeomDataName, SIM_GeometryCopy, SIM_DATA_ADOPT_EXISTING_ON_DELETE );
		GU_DetailHandle lockedPtsDetailHandle = intPtsGeom->lockGeometry();
		GU_DetailHandleAutoWriteLock gdl( lockedPtsDetailHandle );
		GU_Detail *intPtsGdp = gdl.getGdp();
		
		intPtsGdp->destroyPoints( intPtsGdp->getPointRange() );
		
		intPtsGeom->releaseGeometry();
	}  // for i
	
	
	// For each group of continuously connected (colliding) granules (CONN):
	int numConnectedGroups = connectedGroups.entries();
	for ( int i = 0; i < numConnectedGroups; i++ )
	{
		SIM_Relationship* CONN = (SIM_Relationship*)connectedGroups(i);	// Current group of interior and transition granules that are all touching.  A set of connected granules.
		
		for ( int bob = 0; bob < CONN->getGroupEntries(); bob++ )
		{
			SIM_Object* obj = (SIM_Object*)CONN->getGroupObject(bob);
			SIM_EmptyData* data = SIM_DATA_GET( *obj, "GranuleData", SIM_EmptyData );
			SIM_Options& options = data->getData();
			UT_String granuleType;
			options.getOptionString( "granuleType", granuleType );
			cout << obj->getName() << "=" << granuleType << endl;
		}  // for
		
		// Get all the constraint objects in this simulation (the list of ALL solid granular meshes)
		SIM_ObjectArray constraintObjects;
		objects.filter( constraintObjectsFilter, constraintObjects );
		
		// Get all constraints (CONSTR) where some CONSTR_T is the same granule as some CONN_I
		SIM_ObjectArray curConnectedConstraintObjects;		// List to build of all CONSTR connected to CONN
		int numConstraintObjects = constraintObjects.entries();
		for ( int j = 0; j < numConstraintObjects; j++ )
		{
			SIM_Object* curConstraintObj = constraintObjects(j);		// Get the current solid mesh
			
			// Get the group containing all the granules in the current CONSTR (solid mesh)
			SIM_DataFilterByName constraintRelFilter( "glueConstraint*" );
			SIM_ConstDataArray constraintRels;
			curConstraintObj->filterConstRelationships( true, constraintRelFilter, constraintRels );		// Get the "glueConstraint" group from the current solid mesh
			if ( constraintRels.entries() == 0 )
			{
				cout << curConstraintObj->getName() << " has an empty glue object." << endl;
				return;
			}
			SIM_Relationship* constrainedGranulesGrp = (SIM_Relationship*)constraintRels(0);
			if ( !constrainedGranulesGrp )
			{
				cout << curConstraintObj->getName() << " is missing glue relationship." << endl;
				return;
			}
			//cout << curConstraintObj->getName() << endl;
			// For each granule in the current solid granular mesh (CONSTR), check if it is in CONN_I
			int numConstrainedGranules = constrainedGranulesGrp->getGroupEntries();
			for ( int k = 0; k < numConstrainedGranules; k++ )					// This should still be O(n), with n = total number of granules, since each granule should not be constrained to more than one object
			{
				const SIM_Object* curConstrainedGranule = constrainedGranulesGrp->getGroupObject(k);
				if ( CONN->getGroupHasObject( curConstrainedGranule ) )
				{cout << "group " << CONN->getName() << " has obj " << curConstrainedGranule->getName() << endl;
					// Add the constraint object to the list of constraint objects connected to the current CONN, and break
					curConnectedConstraintObjects.add( curConstraintObj );
					break;
				}  // if
			}  // for k
		}  // for j
		
		SIM_Object* CONSTR = NULL;	// The constraint object that will merge in the current
		
		//   If there are shared CONSTR:
		int numConnectedConstraintObjectsToMerge = curConnectedConstraintObjects.entries();
		if ( numConnectedConstraintObjectsToMerge > 0 )
		{
			CONSTR = curConnectedConstraintObjects( 0 );		// If there is only one object, none will be added to it in the following for-loop
			
			if ( numConnectedConstraintObjectsToMerge > 1 )
			{
				// Grab an editable version of CONSTR's mesh geometry to merge the other CONSTRs' geometry with
				SIM_GeometryCopy* solidMeshGeom = SIM_DATA_GET( *CONSTR, solidMeshGeomDataName, SIM_GeometryCopy);
				if ( !solidMeshGeom )
				{
					cout << "Fail. " << solidMeshGeomDataName << " geometry data does not exist on object " << CONSTR->getName() << endl;
				}  // if
				GU_DetailHandle lockedSolidMeshDetailHandle = solidMeshGeom->lockGeometry();
				GU_DetailHandleAutoWriteLock gdl( lockedSolidMeshDetailHandle );
				GU_Detail *solidMeshGdp = gdl.getGdp();
				
				// Get CONSTR's glue relationship
				SIM_DataFilterByName glueRelFilter( "glueConstraint*" );
				SIM_ConstDataArray glueRels;
				CONSTR->filterConstRelationships( true, glueRelFilter, glueRels );
				SIM_Relationship* CONSTRGlueRel = (SIM_Relationship*)glueRels(0);
				if ( !CONSTRGlueRel )
				{
					cout << CONSTR->getName() << " is missing glue relationship." << endl;
					return;
				}
				
				// Connect them all into the first CONSTR obj
				//   The nice thing is that they are guaranteed to not be overlapping, otherwise they would have been merged already.
				//   So we merge all the GEO_Details into the "Geometry" data, then later (in the Houdini network) the new culled geometry can be merged with them.
				int numConstrObjectsToConnect = curConnectedConstraintObjects.entries();
				for ( int j = 1; j < numConstrObjectsToConnect; j++ )
				{
					// Copy the geometry over to the main constraint object
					SIM_Object* curObj = curConnectedConstraintObjects( j );
					GU_DetailHandleAutoReadLock curGdl( curObj->getGeometry()->getGeometry() );
					GU_Detail *curGdp = (GU_Detail*)curGdl.getGdp();
					solidMeshGdp->merge( *curGdp );
					
					// For each granule constrained (glued) to curObj, scoot its constraint over to CONSTR
					SIM_DataFilterByName oldRelFilter( "glueConstraint*" );
					SIM_ConstDataArray oldRels;
					curObj->filterConstRelationships( true, oldRelFilter, oldRels );
					SIM_Relationship* oldRel = (SIM_Relationship*)oldRels(0);
					
					int numGranulesToSwitch = oldRel->getGroupEntries();
					for ( int k = 0; k < numGranulesToSwitch; k++ )					// This should still be O(n), with n = total number of granules, since each granule should not be constrained to more than one object
					{
						CONSTRGlueRel->addGroup( (SIM_Object*)oldRel->getGroupObject(k) );
						CONSTRGlueRel->addAffGroup( (SIM_Object*)oldRel->getGroupObject(k) );
						SIM_DATA_CREATE( *CONSTRGlueRel, "Group",
										SIM_RelationshipGroup,
										SIM_DATA_RETURN_EXISTING );
					}  // for k
					
					// Delete the old CONSTR (remove from CONSTR array then from the engine)
					engine.removeSimulationObject( curObj );
				}  // for j
				
				solidMeshGeom->releaseGeometry();
			}  // if
			
			// Add to a collision relationship.
			SIM_Relationship *collideRel = (SIM_Relationship*)engine.getRelationship( collideRelName );
			if ( !collideRel )
			{
				cout << "could not find " << collideRelName << " for " << CONSTR->getName() << endl;
				return;
			}  // if
			cout << "Collide rel = " << collideRel->getName() << " " << collideRel << endl;
			collideRel->addGroup( CONSTR );
			collideRel->addAffGroup( CONSTR );
			SIM_DATA_CREATE( *collideRel, SIM_RELCOLLIDE_DATANAME,
							SIM_RelationshipCollide,
							SIM_DATA_RETURN_EXISTING );
		}  // if
		//   Else CREATE A NEW GLUE OBJECT (CONSTR):
		else
		{
			engine.setCreatorInfo(getUniqueId(), forOutputIdx);
		
			// Create new CONSTR
			CONSTR = engine.addSimulationObject( doSolveOnCreationFrame );
			char tmp[181]; sprintf( tmp, "%s%d", (char*)constraintObjectsPrefix, CONSTR->getObjectId() );
            UT_String constrName = tmp;
			CONSTR->setName( constrName );
			
			// Add geometry data to the new CONSTR
			SIM_DATA_CREATE( *CONSTR, solidMeshGeomDataName, SIM_SopGeometry, 0 );
			SIM_DATA_CREATE( *CONSTR, interiorGranulePointsGeomDataName, SIM_SopGeometry, 0 );
			SIM_DATA_CREATE( *CONSTR, cullMetasGeomDataName, SIM_SopGeometry, 0 );
			SIM_DATA_CREATE( *CONSTR, cullVolumeGeomDataName, SIM_SopGeometry, 0 );
			SIM_DATA_CREATE( *CONSTR, granuleDataName, SIM_EmptyData, 0 );
			
			engine.setCreatorInfo(getUniqueId(), forOutputIdx);
			
			// Set up a glue relationship
			char tmp2[181]; sprintf( tmp2, "%s%d", "glueConstraint", CONSTR->getObjectId() );
            UT_String glueName = tmp2;
			SIM_Relationship *glueRel = engine.addRelationship( glueName, SIM_DATA_RETURN_EXISTING );
			if ( !glueRel )
				cout << CONSTR->getName() << " was unable to create a glue rel." << endl;
			else
				cout << "Created " << glueRel->getName() << endl;
			glueRel->addGroup( CONSTR );
			glueRel->addAffGroup( CONSTR );      // currObject is the interior granule, since it comes from interiorGranulesFiltered
			SIM_DATA_CREATE( *glueRel, "Group",
							SIM_RelationshipGroup,
							SIM_DATA_RETURN_EXISTING );
			
			// Add to a collision relationship.
			SIM_Relationship *collideRel = (SIM_Relationship*)engine.getRelationship( collideRelName );
			if ( !collideRel )
			{
				cout << "could not find " << collideRelName << " for " << CONSTR->getName() << endl;
				return;
			}  // if
			cout << "Collide rel = " << collideRel->getName() << " " << collideRel << endl;
			collideRel->addGroup( CONSTR );
			collideRel->addAffGroup( CONSTR );
			SIM_DATA_CREATE( *collideRel, SIM_RELCOLLIDE_DATANAME,
							SIM_RelationshipCollide,
							SIM_DATA_RETURN_EXISTING );
		}  // else
		
		if ( CONSTR == NULL )
		{
			cout << "BAD BAD BAD!!  CONSTR is NULL and it shouldn't be.  The end." << endl;
			return;
		}  // if
		
		// Constrain all CONN_T to CONSTR
		//   This is a group of connected (colliding) interior granules
		//   and their accompanying transition granules.  We want to pick
		//   those accompanying transition granules out and constraint them
		//   to their corresponding constraint object CONSTR
		// Meanwhile, add all CONN_I to the CONSTR obj geometry as points (to be used later to union into the geom)
		//   The positions of the interior granules in this connected set of granules
		//   need to be added to the new CONSTR object so that later a solver can
		//   convert those points into the section of mesh surrounding the newly
		//   culled granules (and merging with any solid mesh that it overlaps, which
		//   corresponds to the current CONSTR).
		SIM_GeometryCopy* interiorPointsGeom = SIM_DATA_GET( *CONSTR, interiorGranulePointsGeomDataName, SIM_GeometryCopy);
		if ( !interiorPointsGeom )
			interiorPointsGeom = SIM_DATA_CREATE( *CONSTR, interiorGranulePointsGeomDataName, SIM_GeometryCopy, SIM_DATA_ADOPT_EXISTING_ON_DELETE );
		
		//GU_ConstDetailHandle lockedinteriorPointsDetailHandle = interiorPointsGeom->getGeometry();
		// Get CONSTR's geometry information to be able to add points to the interior point geom
		GU_DetailHandle lockedinteriorPointsDetailHandle = interiorPointsGeom->lockGeometry();
		GU_DetailHandleAutoWriteLock gdl( lockedinteriorPointsDetailHandle );
		GU_Detail *interiorPointsGdp = gdl.getGdp();
		
		// Get CONSTR's glue relationship information to be able to add transition granules to it
		SIM_DataFilterByName glueRelFilter( "glueConstraint*" );
		SIM_ConstDataArray glueRels;
		CONSTR->filterConstRelationships( true, glueRelFilter, glueRels );
		SIM_Relationship* CONSTRGlueRel = (SIM_Relationship*)glueRels(0);
		if ( !CONSTRGlueRel )
		{
			cout << CONSTR->getName() << " is missing glue relationship." << endl;
			return;
		}
		
		// For each granule in the current connected component,
		//   Add it to the interior point positions geometry if it is interior
		//   Constrain it to CONSTR if it is transition
		int numConnectedObjects = CONN->getGroupEntries();
		for ( int j = 0; j < numConnectedObjects; j++ )
		{
			SIM_Object* curGranule = (SIM_Object*)CONN->getGroupObject( j );
			RBD_State *rbdstate = SIM_DATA_GET( *curGranule, "Position", RBD_State );
			SIM_EmptyData* granuleData = SIM_DATA_GET( *curGranule, "GranuleData", SIM_EmptyData );
			
			SIM_Options& options = granuleData->getData();
			UT_String granuleType;
			options.getOptionString( "granuleType", granuleType );
			
			// If it is a transition granule, constrain the transition granule to CONSTR
			if ( granuleType == "transition" )
			{
				rbdstate->setGlueObject( CONSTR->getName() );
				
				CONSTRGlueRel->addGroup( curGranule );
				CONSTRGlueRel->addAffGroup( curGranule );      // currObject is the interior granule, since it comes from interiorGranulesFiltered
			}  // if
			// If it is an interior granule, add a point at its position to the CONSTR interior point geom data.
			else if ( granuleType == "interior" )
			{
				UT_Vector3 pos = rbdstate->getPosition();
				cout << curGranule->getName() << " pos = " << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
				GEO_Point* newPt = interiorPointsGdp->appendPointElement();
				newPt->setPos( pos );
				
			}  // else if
			else
			{
				// Type = none or exterior
				continue;
				//cout << "That's crummy, " << curGranule->getName() << " does not have a proper granule type, " << granuleType << "." << endl;
			}  // else
		}  // for j
		
		interiorPointsGeom->releaseGeometry();
	}  // for i
}  // processObjectsSubclass()

void
DOP_ConstrainNewTransitionGranules::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info)
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
        info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
        info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_ConstrainNewTransitionGranules::getOutputInfoSubclass(int /*outputidx*/, DOP_InOutInfo &info)
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void DOP_ConstrainNewTransitionGranules::CONNECTEDGROUPSPREFIX( UT_String &str, float t )
{
    evalString( str, theConnectedGroupsName.getToken(), 0, t );
}  // CONNECTEDGROUPSPREFIX

void DOP_ConstrainNewTransitionGranules::CONSTRAINTOBJECTSPREFIX( UT_String &str, float t )
{
    evalString( str, theConstraintObjectsName.getToken(), 0, t );
}  // CONSTRAINTOBJECTSPREFIX

void DOP_ConstrainNewTransitionGranules::TRANSITIONOBJECTSPREFIX( UT_String &str, float t )
{
	evalString( str, theTransitionObjectsName.getToken(), 0, t );
}  // GRANULEOBJECTSPREFIX

void DOP_ConstrainNewTransitionGranules::GRANULEOBJECTSPREFIX( UT_String &str, float t )				// Gets the name prefix of the granule objects
{
	evalString( str, theGranuleObjectsName.getToken(), 0, t );
}  // GRANULEOBJECTSPREFIX

void DOP_ConstrainNewTransitionGranules::COLLIDERELNAME( UT_String &str, float t )				// Gets the name prefix of the granule objects
{
	evalString( str, theCollideRelName.getToken(), 0, t );
}  // COLLIDERELNAME

int DOP_ConstrainNewTransitionGranules::SIMULATECREATIONFRAME( float t )						// Check whether the objects should simulate on the frame they were created
{
	return evalInt( theSolveFirstFrame.getToken(), 0, t );
}  // SIMULATECREATIONFRAME

void DOP_ConstrainNewTransitionGranules::SOLIDMESHGEOMETRYDATA( UT_String &str, float t )				// Name of the Geometry data for the solid mesh that is attached to the constraint object
{
	evalString( str, theSolidMeshGeomData.getToken(), 0, t );
}  // SOLIDMESHGEOMETRYDATA

void DOP_ConstrainNewTransitionGranules::INTERIORGRANULEPOINTSGEOMETRYDATA( UT_String &str, float t )	// Name of the Geometry data for the current interior granules that is attached to the constraint object
{
	evalString( str, theInteriorPointsGeomData.getToken(), 0, t );
}  // INTERIORGRANULEPOINTSGEOMETRYDATA

void DOP_ConstrainNewTransitionGranules::CULLMETASGEOMETRYDATA( UT_String &str, float t )				// Name of the Geometry data for the solid mesh that is attached to the constraint object
{
	evalString( str, theCullMetasGeomData.getToken(), 0, t );
}  // CULLMETASGEOMETRYDATA

void DOP_ConstrainNewTransitionGranules::CULLVOLUMEGEOMETRYDATA( UT_String &str, float t )	// Name of the Geometry data for the current interior granules that is attached to the constraint object
{
	evalString( str, theCullVolumeGeomData.getToken(), 0, t );
}  // CULLVOLUMEGEOMETRYDATA

void DOP_ConstrainNewTransitionGranules::GRANULEDATA( UT_String &str, float t )	// Name of the Geometry data for the current interior granules that is attached to the constraint object
{
	evalString( str, theGranuleData.getToken(), 0, t );
}  // GRANULEDATA