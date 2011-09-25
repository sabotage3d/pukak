#include "DOP_ConstrainNewTransitionGranules.h"
#include <UT/UT_DSOVersion.h>
#include <RBD/RBD_State.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_EmptyData.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipGroup.h>
#include <OP/OP_OperatorTable.h>
#include <DOP/DOP_PRMShared.h>
#include <DOP/DOP_InOutInfo.h>
#include <DOP/DOP_Engine.h>

#include "../SIMs/src/SIM_SnowNeighborData.h"

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
static PRM_Name         theGranuleObjectsName( "granuleobjectsprefix", "Granule Objs Prefix" );
static PRM_Name         theSolveFirstFrame( "solvefirstframe", "Solve On Creation Frame" );
static PRM_Name         theSolidMeshGeomData( "solidmeshgeomdata", "Solid Mesh Data" );
static PRM_Name         theInteriorPointsGeomData( "interiorpointsgeomdata", "Interior Geom Data" );

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0 );

PRM_Template
DOP_ConstrainNewTransitionGranules::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template( PRM_INT_J,    1, &DOPactivationName, &DOPactivationDefault ),
    
    // My added parameters
    PRM_Template( PRM_STRING,   1, &theConnectedGroupsName, &defaultNull, 0, 0, 0, 0, 1, "Name prefix of the groups that hold all the interior granules grouped with their neighbors." ),
    PRM_Template( PRM_STRING,   1, &theConstraintObjectsName, &defaultNull, 0, 0, 0, 0, 1, "Name prefix of the existing objects that transition granules are constrained to." ),
	PRM_Template( PRM_STRING,   1, &theGranuleObjectsName, &defaultNull, 0, 0, 0, 0, 1, "Name prefix of all the granule objects (no matter what granuleType label they have)." ),
	PRM_Template( PRM_TOGGLE_J, 1, &theSolveFirstFrame, &defaultNull, 0, 0, 0, 0, 1, "For the newly created objects, this parameter controls whether or not the solver for that object should solve for the object on the timestep in which it was created. Usually this parameter will be turned on if this node is creating objects in the middle of a simulation rather than creating objects for the initial state of the simulation." ),
	//PRM_Template( PRM_TOGGLE_J,  1, &DOPsolvefirstframeName, &defaultZero, 0, 0, 0, 0, 1, "For the newly created objects, this parameter controls whether or not the solver for that object should solve for the object on the timestep in which it was created. Usually this parameter will be turned on if this node is creating objects in the middle of a simulation rather than creating objects for the initial state of the simulation." ),
	PRM_Template( PRM_STRING,   1, &theSolidMeshGeomData, &defaultNull, 0, 0, 0, 0, 1, "Name of the Geometry data in the constraint object that contains the surface mesh geometry of the solid granular mesh." ),
	PRM_Template( PRM_STRING,   1, &theInteriorPointsGeomData, &defaultNull, 0, 0, 0, 0, 1, "Name of the Geometry data in the constraint object that contains the points representing the most recently deleted interior granules." ),
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
DOP_ConstrainNewTransitionGranules::processObjectsSubclass(fpreal time, int,
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
	SIM_ObjectArray constraintObjects;
	UT_String constraintObjectsFilterName = constraintObjectsPrefix;
	constraintObjectsFilterName += "*";
    SIM_DataFilterByName constraintObjectsFilter( constraintObjectsFilterName );
    objects.filter( constraintObjectsFilter, constraintObjects );
	
	
	// Get the prefix of the names of all the granule objects
	UT_String granuleObjectsPrefix;
	GRANULEOBJECTSPREFIX( granuleObjectsPrefix, time );
	
	
	// Get whether or not to solve on the creation frame
	int doSolveOnCreationFrame = SIMULATECREATIONFRAME( time );
	
	
	// Get the name of the Geometry data that contains the solid mesh geometry
	UT_String solidMeshGeomDataName;
	SOLIDMESHGEOMETRYDATA( solidMeshGeomDataName, time );
	
	// Get the name of the Geometry data that contains the solid mesh geometry
	UT_String interiorGranulePointsGeomDataName;
	SOLIDMESHGEOMETRYDATA( interiorGranulePointsGeomDataName, time );
	
	
	if( !isActive(time) )
		return;
	
	// For each group of continuously connected (colliding) granules (CONN):
	int numConnectedGroups = connectedGroups.entries();
	for ( int i = 0; i < numConnectedGroups; i++ )
	{
		SIM_Relationship* CONN = connectedGroups(i);	// Current group of interior and transition granules that are all touching.  A set of connected granules.
		
		// Get all constraints (CONSTR) where some CONSTR_T is the same granule as some CONN_I
		SIM_ObjectArray curConnectedConstraintObjects;		// List to build of all CONSTR connected to CONN
		int numConstraintObjects = constraintObjects.entries();
		for ( int j = 0; j < numConstraintObjects; j++ )
		{
			SIM_Object* curConstraintObj = constraintObjects(j);
			
			// Get the group containing all the granules constrained to the current CONSTR
			SIM_DataFilterByName constraintRelFilter( "glueConstraint*" );
			SIM_ConstDataArray constraintRels;
			curConstraintObj->filterConstRelationships( true, constraintRelFilter, constraintRels );
			SIM_Relationship* constraintRel = (SIM_Relationship*)constraintRels(0);
			
			// For each constrained granule, check if it is in CONN_I
			int numConstrainedGranules = constraintRel->getGroupEntries();
			for ( int k = 0; k < numConstrainedGranules; k++ )					// This should still be O(n), with n = total number of granules, since each granule should not be constrained to more than one object
			{
				const SIM_Object* curConstrainedGranule = constraintRel->getGroupObject(k);
				if ( CONN->getGroupHasObject( curConstrainedGranule ) )
				{
					// Add the constraint object to the list of constraint objects connected to the current CONN, and break
					curConnectedConstraintObjects.add( curConstraintObj );
					break;
				}  // if
			}  // for k
		}  // for j
		
		SIM_Object* CONSTR = NULL;	// The constraint object that will merge in the current
		
		//   If there are shared CONSTR:
		int numConnectedConstraintObjectsToJoin = curConnectedConstraintObjects.entries();
		if ( numConnectedConstraintObjectsToJoin > 0 )
		{
			CONSTR = curConnectedConstraintObjects( 0 );		// If there is only one object, none will be added to it in the following for-loop
			
			if ( numConnectedConstraintObjectsToJoin > 1 )
			{
				//SIM_Geometry* solidMeshGeom = (SIM_Geometry*)CONSTR->getGeometry();
				SIM_GeometryCopy* solidMeshGeom = SIM_DATA_GET( *CONSTR, solidMeshGeomDataName, SIM_GeometryCopy);
				if ( !solidMeshGeom )
				{
					cout << "Fail. " << solidMeshGeomDataName << " geometry data does not exist on object " << CONST->getName() << endl;
				}  // if
				GU_DetailHandle lockedSolidMeshDetailHandle = solidMeshGeom->lockGeometry();
				GU_DetailHandleAutoWriteLock gdl( lockedSolidMeshDetailHandle );
				GU_Detail *solidMeshGdp = gdl.getGdp();
				
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
					
					// Get rid of old CONSTR (remove from CONSTR array then from the engine)
					engine.removeSimulationObject( curObj );
				}  // for j
				
				solidMeshGeom->releaseGeometry();
			}  // if
		}  // if
		//   Else:
		else
		{
			// Create new CONSTR
			CONSTR = engine.addSimulationObject( simFirstFrameBool );
			CONSTR->SIM_DATA_CREATE( *CONSTR, interiorGranulePointsGeomDataName, SIM_Geometry, SIM_DATA_RETURN_EXISTING );
			
			// GIVE IT A POSITION????????????????????????????????????????????????????
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
		GU_DetailHandle lockedinteriorPointsDetailHandle = interiorPointsGeom->lockGeometry();
		GU_DetailHandleAutoWriteLock gdl( lockedinteriorPointsDetailHandle );
		GU_Detail *interiorPointsGdp = gdl.getGdp();
		
		int numConnectedObjects = CONN->getGroupEntries();
		for ( int j = 0; j < numConnectedObjects; j++ )
		{
			SIM_Object* curGranule = CONN->getGroupObject( j );
			RBD_State *rbdstate = SIM_DATA_GET( *curGranule, "Position", RBD_State );
			
			SIM_Options options = rbdstate->getOptions();
			UT_String granuleType;
			options.getOptionString( "granuleType", granuleType );
			
			// If it is a transition granule, constrain the transition granule to CONSTR
			if ( granuleType == "transition" )
			{
				curGranule->setGlueObject( CONSTR->getName() );
			}  // if
			// If it is an interior granule, add a point at its position to the CONSTR interior point geom data.
			else if ( granuleType == "interior" )
			{
				UT_Vector3 pos;
				rbdstate->getPosition( pos );
				
				GEO_Point* newPt = interiorPointsGdp->appendPointElement();
				newPt->setPos( pos );
				
			}  // else if
			else
			{
				cout << "That's crummy, " << curGranule->getName() << " does not have a proper granule type, " << granuleType << "." << endl;
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

void GRANULEOBJECTSPREFIX( UT_String &str, float t )				// Gets the name prefix of the granule objects
{
	evalString( str, theGranuleObjectsName.getToken(), 0, t );
}  // GRANULEOBJECTSPREFIX

int SIMULATECREATIONFRAME( float t )						// Check whether the objects should simulate on the frame they were created
{
	return evalInt( theSolveFirstFrame.getToken(), 0, t );
}  // SIMULATECREATIONFRAME

void SOLIDMESHGEOMETRYDATA( UT_String &str, float t )				// Name of the Geometry data for the solid mesh that is attached to the constraint object
{
	evalString( str, theSolidMeshGeomData.getToken(), 0, t );
}  // SOLIDMESHGEOMETRYDATA

void INTERIORGRANULEPOINTSGEOMETRYDATA( UT_String &str, float t )	// Name of the Geometry data for the current interior granules that is attached to the constraint object
{
	evalString( str, theInteriorPointsGeomData.getToken(), 0, t );
}  // INTERIORGRANULEPOINTSGEOMETRYDATA

