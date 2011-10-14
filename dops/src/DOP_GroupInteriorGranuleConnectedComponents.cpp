#include "DOP_GroupInteriorGranuleConnectedComponents.h"
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
#include <DOP/DOP_Operator.h>
#include <DOP/DOP_Engine.h>


#include <iostream>
#include <vector>



void
newDopOperator(OP_OperatorTable *table)
{
    OP_Operator *op;

    // Create a new DOP_Operator which describes the operator we are
    // building. The parameters to this function are similar to the
    // OP_Operator constructor except for the last parameter, which
    // specifies the number of outputs (up to 4) from this operator.
    op = new DOP_Operator("hdk_groupinteriorgranuleconnectedcomponents", "Group Interior Connected Components",
                          DOP_GroupInteriorGranuleConnectedComponents::myConstructor,
                          DOP_GroupInteriorGranuleConnectedComponents::myTemplateList, 1, 9999, 0,
                          0, 1);
    table->addOperator(op);
}




#define     POINT_CLASS     0
#define     PRIM_CLASS      1
#define     DETAIL_CLASS    2
#define     INT_TYPE        0
#define     FLOAT_TYPE      1
#define     STRING_TYPE     2

//static PRM_Name         theInteriorGranulesGroupName( "interiorgranulesgroupname", "Interior Granules Group" );
static PRM_Name         theNeighborGroupPrefix( "neighborgroupprefix", "Neighbor Group Prefix" );
//static PRM_Name         theShellGranulesGroupName( "shellgranulesgroupname", "Shell Granules Group" );
static PRM_Name         theConnectedComponentGroupPrefix( "connectedcomponentgroupprefix", "Connected Component Group Prefix" );

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0, "" );
static PRM_Default      defaultObjid( 0, "objid" );

PRM_Template
DOP_GroupInteriorGranuleConnectedComponents::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template( PRM_INT_J,    1, &DOPactivationName, &DOPactivationDefault ),
    // Standard group parameter with group menu.
    //PRM_Template( PRM_STRING, 1, &DOPgroupName, &DOPgroupDefault, &DOPgroupMenu ),
    // My added parameters
    //PRM_Template( PRM_STRING,   1, &theShellGranulesGroupName, &defaultNull ),      // The objects read in to this DOP node must belong to the shell granules group
    //PRM_Template( PRM_STRING,   1, &theInteriorGranulesGroupName, &defaultNull ),   // The interior granules group must be provides, to make groups out each group of touching interior and shell granules
    PRM_Template( PRM_STRING,   1, &theNeighborGroupPrefix, &defaultNull, 0, 0, 0, 0, 1, "The mask of the DOP groups that contain the neighboring (contacting) granules of each shell granule." ),
	PRM_Template( PRM_STRING,   1, &theConnectedComponentGroupPrefix, &defaultNull, 0, 0, 0, 0, 1, "The prefix for the name of each group of connected (contacting) granules.  These groups are created by this node." ),
    PRM_Template()
};

OP_Node *
DOP_GroupInteriorGranuleConnectedComponents::myConstructor(OP_Network *net, const char *name,
                                 OP_Operator *op)
{
    return new DOP_GroupInteriorGranuleConnectedComponents(net, name, op);
}

DOP_GroupInteriorGranuleConnectedComponents::DOP_GroupInteriorGranuleConnectedComponents(OP_Network *net, const char *name,
                                     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_GroupInteriorGranuleConnectedComponents::~DOP_GroupInteriorGranuleConnectedComponents()
{
}

void
DOP_GroupInteriorGranuleConnectedComponents::processObjectsSubclass(fpreal time, int,
                                          const SIM_ObjectArray &objects,
                                          DOP_Engine &engine)
{
    SIM_ObjectArray      interiorgranules;
    UT_String            group;
    int                  i;         //, inputindex;
    
	// Get the neighbor groups
	UT_String neighborGroupPrefix;
	NEIGHBORGROUPPREFIX( neighborGroupPrefix, time );
    neighborGroupPrefix.strip("*");
	
	SIM_ConstDataArray neighborGroups;
	UT_String neighborGroupFilterName = neighborGroupPrefix;		// groupMask is the group prefix name (e.g. neighborsOf*)
	neighborGroupFilterName += "*";
	SIM_DataFilterByName neighborGroupFilter( neighborGroupFilterName );
	engine.filterConstRelationships( neighborGroupFilter, neighborGroups );
    
    // Get the prefix of the group name for each group of connected (colliding) granules we will create
    UT_String connectedComponentGroupPrefix;
    CONNECTEDCOMPONENTGROUPPREFIX( connectedComponentGroupPrefix, time );
    
	// Delete the old connected component groups to make room for the new
    UT_String deleteOldCCFilterString = connectedComponentGroupPrefix;
	deleteOldCCFilterString += "*";
	SIM_DataFilterByName oldCCGroupFilter( deleteOldCCFilterString );
	SIM_ConstDataArray deleteOldCCGroups;
	engine.filterConstRelationships( oldCCGroupFilter, deleteOldCCGroups );
	int numDeleteGroups = deleteOldCCGroups.entries();
	for ( int i = 0; i < numDeleteGroups; i++ )
	{
		engine.removeRelationship( (SIM_Relationship*)deleteOldCCGroups[i] );
	}  // for i
	
	SIM_ConstDataArray connectedComponentGroups;
	UT_String connectedComponentGroupFilterName = connectedComponentGroupPrefix;
	connectedComponentGroupFilterName += "*";
	SIM_DataFilterByName connectedComponentGroupFilter( connectedComponentGroupFilterName );
    engine.filterConstRelationships( connectedComponentGroupFilter, connectedComponentGroups );
	
	int numConnectedComponentGroups = 0;
	if ( connectedComponentGroups.entries() > 0 )
	{
		UT_String lastGroupName = (UT_String)( ((SIM_Relationship*)connectedComponentGroups[connectedComponentGroups.entries()-1])->getName() );
		lastGroupName.strip( connectedComponentGroupPrefix );
		int numConnectedComponentGroups = lastGroupName.toInt() + 1;
	}  // if
	
	
	int numNeighborGroups = neighborGroups.entries();
	for ( int i = 0 ; i < numNeighborGroups; i++ )
	{
		SIM_Relationship* curNeighborGroup =(SIM_Relationship*)neighborGroups[i];
		
		bool isConnected = false;
		SIM_ConstDataArray curConnectedGroups;
		
		int numNeighbors = curNeighborGroup->getGroupEntries();
		int connectedComponentGroupsSize = connectedComponentGroups.entries();
		for ( int j = 0; j < connectedComponentGroupsSize; j++ )
		{
			SIM_Relationship* curCCGroup = (SIM_Relationship*)connectedComponentGroups[j];
			for ( int n = 0; n < numNeighbors; n++ )
			{
				SIM_Object* currObject = (SIM_Object*)curNeighborGroup->getGroupObject(n);
				if ( curCCGroup->getGroupHasObject( currObject ) )
				{
					isConnected = true;
					curConnectedGroups.append( curCCGroup );
					break;
				}  // if
			}  // for n
		}  // for j
		
		if ( !isConnected )		// Create a new connected component group for this granule
		{
			char tmp[181];
            sprintf( tmp, "%s%d", (char*)connectedComponentGroupPrefix, numConnectedComponentGroups++ );
            UT_String connectedComponentGroupName = tmp;
			SIM_Relationship* newConnectedComponentGroup = engine.addRelationship( connectedComponentGroupName, SIM_DATA_RETURN_EXISTING );
			for ( int n = 0; n < numNeighbors; n++ )
			{
				SIM_Object* currObject = (SIM_Object*)curNeighborGroup->getGroupObject(n);
				newConnectedComponentGroup->addGroup( currObject );      // currObject is in the shell granules group, since it comes from interiorgranules
				SIM_DATA_CREATE( *newConnectedComponentGroup, SIM_RELGROUP_DATANAME,
								SIM_RelationshipGroup,
								SIM_DATA_RETURN_EXISTING );
			}  // for
			
			connectedComponentGroups.append( newConnectedComponentGroup );
		}  // if
		else if ( curConnectedGroups.entries() == 1 )		// Add the current neighbors to the connected group
		{
			SIM_Relationship* curCCGroup = (SIM_Relationship*)curConnectedGroups[0];
			for ( int n = 0; n < numNeighbors; n++ )
			{
				SIM_Object* currObject = (SIM_Object*)curNeighborGroup->getGroupObject(n);
				if ( !curCCGroup->getGroupHasObject( currObject ) )
				{
					curCCGroup->addGroup( currObject );      // currObject is in the shell granules group, since it comes from interiorgranules
					SIM_DATA_CREATE( *curCCGroup, SIM_RELGROUP_DATANAME,
									SIM_RelationshipGroup,
									SIM_DATA_RETURN_EXISTING );
				}  // if
			}  // for n
		}  // else if
		else			// There are multiple connected component groups to join together
		{
			char tmp[181];
            sprintf( tmp, "%s%d", (char*)connectedComponentGroupPrefix, numConnectedComponentGroups++ );
            UT_String connectedComponentGroupName = tmp;
			SIM_Relationship* newConnectedComponentGroup = engine.addRelationship( connectedComponentGroupName, SIM_DATA_RETURN_EXISTING );
			
			int numCurConnectedGroups = curConnectedGroups.entries();
			for ( int i = 0; i < numCurConnectedGroups; i++ )
			{
				SIM_Relationship* curConnectedGroup = (SIM_Relationship*)curConnectedGroups[i];
				
				int numGroupObjects = curConnectedGroup->getGroupEntries();
				for ( int j = 0; j < numGroupObjects; j++ )
				{
					SIM_Object* currObject = (SIM_Object*)curConnectedGroup->getGroupObject(j);
					newConnectedComponentGroup->addGroup( currObject );      // currObject is in the shell granules group, since it comes from interiorgranules
					SIM_DATA_CREATE( *newConnectedComponentGroup, SIM_RELGROUP_DATANAME,
									SIM_RelationshipGroup,
									SIM_DATA_RETURN_EXISTING );
				}  // for j
				
				//connectedComponentGroups.remove( curConnectedGroup );
				connectedComponentGroups.findAndRemove( curConnectedGroup );
			}  // for i
			
			// Delete the old component groups
			for ( int i = 0; i < numCurConnectedGroups; i++ )
			{
				SIM_Relationship* curConnectedGroup = (SIM_Relationship*)curConnectedGroups[i];
				engine.removeRelationship( curConnectedGroup );
			}  // for i
			
			for ( int n = 0; n < numNeighbors; n++ )
			{
				SIM_Object* currObject = (SIM_Object*)curNeighborGroup->getGroupObject(n);
				if ( !newConnectedComponentGroup->getGroupHasObject( currObject ) )
				{
					newConnectedComponentGroup->addGroup( currObject );      // currObject is in the shell granules group, since it comes from interiorgranules
					SIM_DATA_CREATE( *newConnectedComponentGroup, SIM_RELGROUP_DATANAME,
									SIM_RelationshipGroup,
									SIM_DATA_RETURN_EXISTING );
				}  // if
			}  // for n
			
			connectedComponentGroups.append( newConnectedComponentGroup );
		}  // else
	}  // for i
    
}  // processObjectsSubclass()

void
DOP_GroupInteriorGranuleConnectedComponents::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info)
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
        info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
        info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_GroupInteriorGranuleConnectedComponents::getOutputInfoSubclass(int /*outputidx*/, DOP_InOutInfo &info)
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void
DOP_GroupInteriorGranuleConnectedComponents::GROUP(UT_String &str, fpreal t)
{
    evalString(str, DOPgroupName.getToken(), 0, t);
}

//void DOP_GroupInteriorGranuleConnectedComponents::INTERIORGRANULESGROUPNAME( UT_String &str, float t )
//{
//    evalString( str, theInteriorGranulesGroupName.getToken(), 0, t );
//}  // INTERIORGRANULESGROUPNAME

//void DOP_GroupInteriorGranuleConnectedComponents::SHELLGRANULESGROUPNAME( UT_String &str, float t )
//{
//    evalString( str, theShellGranulesGroupName.getToken(), 0, t );
//}  // SHELLGRANULESGROUPNAME

void DOP_GroupInteriorGranuleConnectedComponents::NEIGHBORGROUPPREFIX( UT_String &str, float t )
{
    evalString( str, theNeighborGroupPrefix.getToken(), 0, t );
}  // NEIGHBORGROUPPREFIX

void DOP_GroupInteriorGranuleConnectedComponents::CONNECTEDCOMPONENTGROUPPREFIX( UT_String &str, float t )
{
    evalString( str, theConnectedComponentGroupPrefix.getToken(), 0, t );
}  // CONNECTEDCOMPONENTGROUPPREFIX


