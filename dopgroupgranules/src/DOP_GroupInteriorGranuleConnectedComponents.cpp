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

#include "../SIM_SnowBulletSolver/src/SIM_SnowNeighborData.h"

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
    
    // Get the name of the interior granules group, based on the Interior Granules Group parameter input
    //   The interior granules are granules that are ready to be culled, whose outer neighbors we will set
    //   to be shell granules.
    //UT_String interiorGranulesGroupName;
    //INTERIORGRANULESGROUPNAME( interiorGranulesGroupName, time );
    //if ( interiorGranulesGroupName == "" )
    //    return;
    //const SIM_Relationship* interiorGranulesGroup = engine.getRelationship( interiorGranulesGroupName );
    //if ( !interiorGranulesGroup )
    //    return;
    
    // Get the name for the shell granules group that will be created,
    //   based on the Shell Granules Group parameter input.
    //   This group will be populated with exterior granules that border the pockets of interior granules.
    //UT_String shellGranulesGroupName;
    //SHELLGRANULESGROUPNAME( shellGranulesGroupName, time );
    //if ( shellGranulesGroupName == "" )
    //    return;
    //const SIM_Relationship* shellGranulesGroup = engine.getRelationship( shellGranulesGroupName );

    // Grab the group string and filter our incoming objects using that
    // string. This narrows down the set of objects that we actually want
    // to operate on. The filtered interiorgranules array will contain only those objects
    // from the original objects array that match the supplied string.
    //GROUP(group, time);
    //SIM_DataFilterRootData       filter(interiorGranulesGroupName);
    //objects.filter(filter, interiorgranules);
	
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
    //GB_AttributeRef objidAttrOffset = meshGdp->findPointAttrib( "objid", sizeof(int), GB_ATTRIB_INT );
    
    
    // Delete all the groups with prefix neighborGroupPrefix
    //SIM_ConstDataArray oldNeighborGroups;
    //UT_String filterGroupName = neighborGroupPrefix;
    //filterGroupName += "*";
    //SIM_DataFilterByName groupFilter( filterGroupName );
    //engine.filterConstRelationships( groupFilter, oldNeighborGroups );
    //
    //int numOldNeighborGroups = oldNeighborGroups.entries();
    //for ( int n = 0; n < numOldNeighborGroups; n++ )
    //{
    //    engine.removeRelationship( (SIM_Relationship*)oldNeighborGroups[n] );
    //}  // for n
	
	int numConnectedComponentGroups = 0;
	SIM_ConstDataArray connectedComponentGroups;
	
	int numNeighborGroups = neighborGroups.entries();
	for ( int i = 0 ; i < numNeighborGroups; i++ )
	{
		SIM_Relationship* curNeighborGroup =(SIM_Relationship*)neighborGroups[i];
		
		bool isConnected = false;
		SIM_ConstDataArray curConnectedGroups;
		
		int numNeighbors = curNeighborGroup->getGroupEntries();
		int numConnectedComponentGroups = connectedComponentGroups.entries();
		for ( int j = 0; j < numConnectedComponentGroups; j++ )
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
				
				connectedComponentGroups.remove( curConnectedGroup );
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
    
    // Loop through all the objects that passed the filter.
	/*int numInteriorGranules = interiorgranules.entries();
    for( i = 0; i < numInteriorGranules; i++ )
    {
        // Set information about the object we are going to process.
        // The first argument is the index of the current object within the
        // full list of objects we are going to process. The second
        // argument is the total number of objects we are going to process.
        // The last argument is a pointer to the actual object we are
        // processing.
        setCurrentObject( i, numInteriorGranules, interiorgranules(i) );

        // The isActive function checks both the bypass flag and the
        // activation parameter on the node (if there is one, which there
        // is in this case). We call this function after calling
        // setCurrentObject and we call it for each object in case the
        // activation parameter uses some object-specific variables
        // like OBJID in an expression.
        if( isActive(time) )
        {
            // Get the current object's objid attribute.
            SIM_Object* currObject = interiorgranules(i);
            int objid = currObject->getObjectId();
            //cout << currObject->getName() << endl;
            char tmp[181];
            sprintf( tmp, "%s%d", (char*)connectedComponentGroupPrefix, objid );
            UT_String connectedComponentGroupName = tmp;
            
            SIM_SnowNeighborData* currNeighborData = SIM_DATA_GET( *currObject, "Bullet Neighbor Data", SIM_SnowNeighborData );
            if ( currNeighborData )
            {
                // For each neighboring object to the current object, see if the neighbor is an interior granule
                int numNeighbors = currNeighborData->getNumNeighbors();
				
                SIM_Relationship *connectedComponentGroup = engine.addRelationship( connectedComponentGroupName, SIM_DATA_RETURN_EXISTING );
                connectedComponentGroup->addGroup( currObject );      // currObject is in the shell granules group, since it comes from interiorgranules
                SIM_DATA_CREATE( *connectedComponentGroup, SIM_RELGROUP_DATANAME,
                                SIM_RelationshipGroup,
                                SIM_DATA_RETURN_EXISTING );
                
				// For each neighbor of the current interior granule, add it to a connected component group that
				//   it is collided with, or if it is not connected to any of the current connected component groups,
				//   create a new connected component group
                for ( int n = 0; n < numNeighbors; n++ )
                {
                    // Add the neighbor to the current shell granule's group.
                    int neighborId = currNeighborData->getNeighborId( n );
                    SIM_Object* neighborObject = (SIM_Object*)engine.getSimulationObjectFromId( neighborId );
                    if ( !neighborObject )
                        continue;
                    
                    //neighborGroup = engine.addRelationship( neighborGroupName, SIM_DATA_RETURN_EXISTING );
                    connectedComponentGroup->addGroup( neighborObject );
                    SIM_DATA_CREATE( *neighborGroup, SIM_RELGROUP_DATANAME,
                                    SIM_RelationshipGroup,
                                    SIM_DATA_RETURN_EXISTING );
                    
                    */
                    // If the neighbor is an interior granule, put it in a group with all its shell granule neighbors
                    /*if ( interiorGranulesGroup->getGroupHasObject( neighborObject ) )
                    {
                        // Create the neighbor group for this interior granule
                        char tmp[181];
                        sprintf( tmp, "%s%d", (char*)neighborGroupPrefix, neighborId );
                        UT_String interiorNeighborGroupName = tmp;
                        //cout << "  interiorNeighborGroupName = " << interiorNeighborGroupName << endl;
                        
                        SIM_Relationship *interiorNeighborGroup = engine.addRelationship( interiorNeighborGroupName, SIM_DATA_RETURN_EXISTING );
                        interiorNeighborGroup->addGroup( neighborObject );
                        SIM_DATA_CREATE( *interiorNeighborGroup, SIM_RELGROUP_DATANAME,
                                        SIM_RelationshipGroup,
                                        SIM_DATA_RETURN_EXISTING );
                        
                        SIM_SnowNeighborData* currInteriorData = SIM_DATA_GET( *neighborObject, "Bullet Neighbor Data", SIM_SnowNeighborData );
                        if ( currInteriorData )
                        {
                            int numInteriorNeighbors = currInteriorData->getNumNeighbors();
                            for ( int in = 0; in < numInteriorNeighbors; in++ )
                            {
                                int interiorNeighborId = currInteriorData->getNeighborId( in );
                                SIM_Object* interiorNeighborObject = (SIM_Object*)engine.getSimulationObjectFromId( interiorNeighborId );
                                if ( !interiorNeighborObject )
                                    continue;
                                    
                                if ( shellGranulesGroup->getGroupHasObject( interiorNeighborObject ) )
                                {
                                    interiorNeighborGroup->addGroup( interiorNeighborObject );
                                    SIM_DATA_CREATE( *interiorNeighborGroup, SIM_RELGROUP_DATANAME,
                                                    SIM_RelationshipGroup,
                                                    SIM_DATA_RETURN_EXISTING );
                                }  // if
                            }  // for in
                        }  // if
                    }  // if*/
                    
                    /*
                    if ( interiorGranulesGroup->getGroupHasObject( neighborObj ) )
                    {
                        SIM_Relationship *shellGroup = engine.addRelationship( shellGranulesGroupName, SIM_DATA_RETURN_EXISTING );
                        if ( shellGroup )
                        {
                            shellGroup->addGroup( currObject );
                            SIM_DATA_CREATE( *shellGroup, SIM_RELGROUP_DATANAME,
                                            SIM_RelationshipGroup,
                                            SIM_DATA_RETURN_EXISTING );
                            break;
                        }  // if
                    }  // if
                    */
    //            }  // for n
    //        }  // if
    //    }  // if
    //}  // for each object
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


