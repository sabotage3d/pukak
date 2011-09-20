#include "DOP_GroupNewTransitionGranules.h"
#include <UT/UT_DSOVersion.h>
#include <RBD/RBD_State.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_EmptyData.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipGroup.h>
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
    op = new DOP_Operator("hdk_groupnewtransitiongranules", "Group New Transition Granules",
                          DOP_GroupNewTransitionGranules::myConstructor,
                          DOP_GroupNewTransitionGranules::myTemplateList, 1, 9999, 0,
                          0, 1);
    table->addOperator(op);
}




#define     POINT_CLASS     0
#define     PRIM_CLASS      1
#define     DETAIL_CLASS    2
#define     INT_TYPE        0
#define     FLOAT_TYPE      1
#define     STRING_TYPE     2

static PRM_Name         theNewGroupName( "newgroupname", "New Group" );
static PRM_Name         theInteriorGranulesGroupName( "interiorgranulesgroupname", "Interior Granules Group" );
//static PRM_Name         theExteriorGranulesGroupName( "exteriorgranulesgroupname", "Exterior Granules Group" );

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0, "" );
static PRM_Default      defaultObjid( 0, "objid" );

PRM_Template
DOP_GroupNewTransitionGranules::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template( PRM_INT_J,    1, &DOPactivationName, &DOPactivationDefault ),
    // Standard group parameter with group menu.
    //PRM_Template( PRM_STRING, 1, &DOPgroupName, &DOPgroupDefault, &DOPgroupMenu ),
    // My added parameters
    PRM_Template( PRM_STRING,   1, &theNewGroupName, &defaultNull ),      // The name of the group to create for grouping the new transition granules (granules neighboring interior granules)
    PRM_Template( PRM_STRING,   1, &theInteriorGranulesGroupName, &defaultNull ),   // The interior granules group must be provided, of which neighboring transition granules will be found
	//PRM_Template( PRM_STRING,   1, &theExteriorGranulesGroupName, &defaultNull ),   // The interior granules group must be provided, of which neighboring transition granules will be found
    PRM_Template()
};

OP_Node *
DOP_GroupNewTransitionGranules::myConstructor(OP_Network *net, const char *name,
                                 OP_Operator *op)
{
    return new DOP_GroupNewTransitionGranules(net, name, op);
}

DOP_GroupNewTransitionGranules::DOP_GroupNewTransitionGranules(OP_Network *net, const char *name,
                                     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_GroupNewTransitionGranules::~DOP_GroupNewTransitionGranules()
{
}

void
DOP_GroupNewTransitionGranules::processObjectsSubclass(fpreal time, int,
                                          const SIM_ObjectArray &objects,
                                          DOP_Engine &engine)
{
    UT_String            group;
	
    // Get the name for the transition granules group that will be created,
    //   based on the Shell Granules Group parameter input.
    //   This group will be populated with exterior granules that border the pockets of interior granules.
    UT_String newGroupName;
    NEWGROUPNAME( newGroupName, time );
    if ( newGroupName == "" )
        return;
    SIM_Relationship* newGroup = engine.addRelationship( newGroupName, SIM_DATA_RETURN_EXISTING );
    
    // Get the name of the interior granules group, based on the Interior Granules Group parameter input
    //   The interior granules are granules that are ready to be culled, whose outer neighbors we will set
    //   to be shell granules.
    UT_String interiorGranulesGroupName;
    INTERIORGRANULESGROUPNAME( interiorGranulesGroupName, time );
    if ( interiorGranulesGroupName == "" )
        return;
	const SIM_Relationship* interiorGranulesGroup = engine.getRelationship( interiorGranulesGroupName );
    if ( !interiorGranulesGroup )
        return;
	
    SIM_DataFilterRootData  interiorGranulesFilter( interiorGranulesGroupName );
	SIM_ObjectArray interiorGranulesFiltered;
    objects.filter( interiorGranulesFilter, interiorGranulesFiltered );
	
	
    // Get the name of the interior granules group, based on the Interior Granules Group parameter input
    //   The interior granules are granules that are ready to be culled, whose outer neighbors we will set
    //   to be shell granules.
    //UT_String exteriorGranulesGroupName;
    //EXTERIORGRANULESGROUPNAME( exteriorGranulesGroupName, time );
    //if ( exteriorGranulesGroupName == "" )
    //    return;
    //const SIM_Relationship* exteriorGranulesGroup = engine.getRelationship( exteriorGranulesGroupName );
    //if ( !exteriorGranulesGroup )
    //    return;
    
    
    // Loop through all the objects that passed the interiorGranulesFilter.
	int numInteriorGranules = interiorGranulesFiltered.entries();
    for( int i = 0; i < numInteriorGranules; i++ )
    {
        // Set information about the object we are going to process.
        // The first argument is the index of the current object within the
        // full list of objects we are going to process. The second
        // argument is the total number of objects we are going to process.
        // The last argument is a pointer to the actual object we are
        // processing.
        setCurrentObject( i, numInteriorGranules, interiorGranulesFiltered(i) );
		
		// Get the current object's objid attribute.
		SIM_Object* currObject = interiorGranulesFiltered(i);
		int objid = currObject->getObjectId();
		//cout << currObject->getName() << endl;
		
		// Get the impacts data
		SIM_Impacts* impactsData = SIM_DATA_GET( *currObject, "Impacts", SIM_Impacts );
		if ( !impactsData )
			continue;
		
		int numImpacts = impactsData->getNumImpacts();
		for ( int j = 0; j < numImpacts; j++ )
		{
			int otherObjid = impactsData->getOtherObjId( j );
			SIM_Object* neighborObj = (SIM_Object*)engine.getSimulationObjectFromId( otherObjid );
			
			if ( !(interiorGranulesGroup->getGroupHasObject( neighborObj )) )
			{
				newGroup->addGroup( neighborObj );
				break;
			}  // if
		}  // for j
	}  // for i
}  // processObjectsSubclass()

void
DOP_GroupNewTransitionGranules::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info)
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
        info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
        info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_GroupNewTransitionGranules::getOutputInfoSubclass(int /*outputidx*/, DOP_InOutInfo &info)
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void
DOP_GroupNewTransitionGranules::NEWGROUPNAME(UT_String &str, fpreal t)
{
    evalString(str, theNewGroupName.getToken(), 0, t);
}  // NEWGROUPNAME

void DOP_GroupNewTransitionGranules::INTERIORGRANULESGROUPNAME( UT_String &str, float t )
{
    evalString( str, theInteriorGranulesGroupName.getToken(), 0, t );
}  // INTERIORGRANULESGROUPNAME

//void DOP_GroupNewTransitionGranules::EXTERIORGRANULESGROUPNAME( UT_String &str, float t )
//{
//    evalString( str, theExteriorGranulesGroupName.getToken(), 0, t );
//}  // EXTERIORGRANULESGROUPNAME

