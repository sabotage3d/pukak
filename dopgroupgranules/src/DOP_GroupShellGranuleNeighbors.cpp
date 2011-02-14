#include "DOP_GroupShellGranuleNeighbors.h"
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



void
newDopOperator(OP_OperatorTable *table)
{
    OP_Operator *op;

    // Create a new DOP_Operator which describes the operator we are
    // building. The parameters to this function are similar to the
    // OP_Operator constructor except for the last parameter, which
    // specifies the number of outputs (up to 4) from this operator.
    op = new DOP_Operator("hdk_groupshellgranuleneighbors", "Group Shell Granule Neighbors",
                          DOP_GroupShellGranuleNeighbors::myConstructor,
                          DOP_GroupShellGranuleNeighbors::myTemplateList, 1, 9999, 0,
                          0, 1);
    table->addOperator(op);
}




#define     POINT_CLASS     0
#define     PRIM_CLASS      1
#define     DETAIL_CLASS    2
#define     INT_TYPE        0
#define     FLOAT_TYPE      1
#define     STRING_TYPE     2

static PRM_Name         theInteriorGranulesGroupName( "interiorgranulesgroupname", "Interior Granules Group" );
static PRM_Name         theShellGranulesGroupName( "shellgranulesgroupname", "Shell Granules Group" );
static PRM_Name         theNeighborGroupPrefix( "neighborgroupprefix", "Neighbor Group Prefix" );

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0, "" );
static PRM_Default      defaultObjid( 0, "objid" );

PRM_Template
DOP_GroupShellGranuleNeighbors::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template( PRM_INT_J,     1, &DOPactivationName, &DOPactivationDefault ),
    // Standard group parameter with group menu.
    //PRM_Template( PRM_STRING,    1, &DOPgroupName, &DOPgroupDefault, &DOPgroupMenu ),
    // My added parameters
    PRM_Template( PRM_STRING,   1, &theShellGranulesGroupName, &defaultNull ),      // The objects read in to this DOP node must belong to the shell granules group
    PRM_Template( PRM_STRING,   1, &theInteriorGranulesGroupName, &defaultNull ),   // The interior granules group must be provides, to make neighbor groups out of neighbors of interior granules that are neighboring shell granules
    PRM_Template( PRM_STRING,   1, &theNeighborGroupPrefix, &defaultNull ),
    PRM_Template()
};

OP_Node *
DOP_GroupShellGranuleNeighbors::myConstructor(OP_Network *net, const char *name,
                                 OP_Operator *op)
{
    return new DOP_GroupShellGranuleNeighbors(net, name, op);
}

DOP_GroupShellGranuleNeighbors::DOP_GroupShellGranuleNeighbors(OP_Network *net, const char *name,
                                     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_GroupShellGranuleNeighbors::~DOP_GroupShellGranuleNeighbors()
{
}

void
DOP_GroupShellGranuleNeighbors::processObjectsSubclass(fpreal time, int,
                                          const SIM_ObjectArray &objects,
                                          DOP_Engine &engine)
{
    SIM_ObjectArray      filtered;
    UT_String            group;
    int                  i;         //, inputindex;
    
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
    
    // Get the name for the shell granules group that will be created,
    //   based on the Shell Granules Group parameter input.
    //   This group will be populated with exterior granules that border the pockets of interior granules.
    UT_String shellGranulesGroupName;
    SHELLGRANULESGROUPNAME( shellGranulesGroupName, time );
    if ( shellGranulesGroupName == "" )
        return;
    const SIM_Relationship* shellGranulesGroup = engine.getRelationship( shellGranulesGroupName );

    // Grab the group string and filter our incoming objects using that
    // string. This narrows down the set of objects that we actually want
    // to operate on. The filtered array will contain only those objects
    // from the original objects array that match the supplied string.
    //GROUP(group, time);
    SIM_DataFilterRootData       filter(shellGranulesGroupName);
    objects.filter(filter, filtered);
    
    // Get the prefix of the group name for each group of neighbors we will create
    UT_String neighborGroupPrefix;
    NEIGHBORGROUPPREFIX( neighborGroupPrefix, time );
    //GB_AttributeRef objidAttrOffset = meshGdp->findPointAttrib( "objid", sizeof(int), GB_ATTRIB_INT );
    
    
    // Delete all the groups with prefix neighborGroupPrefix
    SIM_ConstDataArray oldNeighborGroups;
    UT_String filterGroupName = neighborGroupPrefix;
    filterGroupName += "*";
    SIM_DataFilterByName groupFilter( filterGroupName );
    engine.filterConstRelationships( groupFilter, oldNeighborGroups );
    
    int numOldNeighborGroups = oldNeighborGroups.entries();
    for ( int n = 0; n < numOldNeighborGroups; n++ )
    {
        engine.removeRelationship( (SIM_Relationship*)oldNeighborGroups[n] );
    }  // for n
    
    // Loop through all the objects that passed the filter.
    for( i = 0; i < filtered.entries(); i++ )
    {
        // Set information about the object we are going to process.
        // The first argument is the index of the current object within the
        // full list of objects we are going to process. The second
        // argument is the total number of objects we are going to process.
        // The last argument is a pointer to the actual object we are
        // processing.
        setCurrentObject( i, filtered.entries(), filtered(i) );

        // The isActive function checks both the bypass flag and the
        // activation parameter on the node (if there is one, which there
        // is in this case). We call this function after calling
        // setCurrentObject and we call it for each object in case the
        // activation parameter uses some object-specific variables
        // like OBJID in an expression.
        if( isActive(time) )
        {
            // Get the current object's objid attribute.
            SIM_Object* currObject = filtered(i);
            int objid = currObject->getObjectId();
            
            char tmp[181];
            sprintf( tmp, "%s%d", (char*)neighborGroupPrefix, objid );
            UT_String neighborGroupName = tmp;
            
            SIM_SnowNeighborData* currNeighborData = SIM_DATA_GET( *currObject, "Bullet Neighbor Data", SIM_SnowNeighborData );
            if ( currNeighborData )
            {
                // For each neighboring object to the current object, see if the neighbor is an interior granule
                int numNeighbors = currNeighborData->getNumNeighbors();
                
                // If none of the shell granule's neighbors are interior granules, continue on to the next shell granule
                int numShellNeighborsThatAreInteriorGranules = 0;
                for ( int n = 0; n < numNeighbors; n++ )
                {
                    int neighborId = currNeighborData->getNeighborId( n );
                    SIM_Object* neighborObject = (SIM_Object*)engine.getSimulationObjectFromId( neighborId );
                    if ( !neighborObject )
                        continue;
                    
                    if ( interiorGranulesGroup->getGroupHasObject( neighborObject ) )
                    {
                        numShellNeighborsThatAreInteriorGranules++;
                    }  // if
                } // for
                
                if ( numShellNeighborsThatAreInteriorGranules == 0 )
                    continue;
                //cout << numShellNeighborsThatAreInteriorGranules << " interior neighbors found." << endl;
                //cout << "creating group " << neighborGroupName << endl;
                SIM_Relationship *neighborGroup = engine.addRelationship( neighborGroupName, SIM_DATA_RETURN_EXISTING );
                neighborGroup->addGroup( currObject );      // currObject is in the shell granules group, since it comes from filtered
                SIM_DATA_CREATE( *neighborGroup, SIM_RELGROUP_DATANAME,
                                SIM_RelationshipGroup,
                                SIM_DATA_RETURN_EXISTING );
                
                for ( int n = 0; n < numNeighbors; n++ )
                {
                    // Add the neighbor to the current shell granule's group.
                    int neighborId = currNeighborData->getNeighborId( n );
                    SIM_Object* neighborObject = (SIM_Object*)engine.getSimulationObjectFromId( neighborId );
                    if ( !neighborObject )
                        continue;
                    if ( !interiorGranulesGroup->getGroupHasObject(neighborObject) && !shellGranulesGroup->getGroupHasObject(neighborObject) )
                        continue;
                    
                    //neighborGroup = engine.addRelationship( neighborGroupName, SIM_DATA_RETURN_EXISTING );
                    neighborGroup->addGroup( neighborObject );
                    SIM_DATA_CREATE( *neighborGroup, SIM_RELGROUP_DATANAME,
                                    SIM_RelationshipGroup,
                                    SIM_DATA_RETURN_EXISTING );
                    
                    
                    // If the neighbor is an interior granule, put it in a group with all its shell granule neighbors
                    if ( interiorGranulesGroup->getGroupHasObject( neighborObject ) )
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
                    }  // if
                    
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
                }  // for n
            }  // if
        }  // if
    }  // for each object
}  // processObjectsSubclass()

void
DOP_GroupShellGranuleNeighbors::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info)
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
        info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
        info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_GroupShellGranuleNeighbors::getOutputInfoSubclass(int /*outputidx*/, DOP_InOutInfo &info)
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void
DOP_GroupShellGranuleNeighbors::GROUP(UT_String &str, fpreal t)
{
    evalString(str, DOPgroupName.getToken(), 0, t);
}

void DOP_GroupShellGranuleNeighbors::INTERIORGRANULESGROUPNAME( UT_String &str, float t )
{
    evalString( str, theInteriorGranulesGroupName.getToken(), 0, t );
}  // INTERIORGRANULESGROUPNAME

void DOP_GroupShellGranuleNeighbors::SHELLGRANULESGROUPNAME( UT_String &str, float t )
{
    evalString( str, theShellGranulesGroupName.getToken(), 0, t );
}  // SHELLGRANULESGROUPNAME

void DOP_GroupShellGranuleNeighbors::NEIGHBORGROUPPREFIX( UT_String &str, float t )
{
    evalString( str, theNeighborGroupPrefix.getToken(), 0, t );
}  // NEIGHBORGROUPPREFIX


