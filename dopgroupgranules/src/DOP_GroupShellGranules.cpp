#include "DOP_GroupShellGranules.h"
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
    op = new DOP_Operator("hdk_groupshellgranules", "Group Shell Granules",
                          DOP_GroupShellGranules::myConstructor,
                          DOP_GroupShellGranules::myTemplateList, 1, 9999, 0,
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
static PRM_Name         theMeshGranulesGroupName( "meshgranulesgroup", "Mesh Granules Group" );
static PRM_Name         theShellGranulesForNewMeshGroupName( "shellgranulesfornewmeshgroupname", "Shells for New Mesh Group" );   // Shell granules that will not connect into an already existing granular mesh but will help form a new one

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0, "" );
static PRM_Default      defaultObjid( 0, "objid" );

PRM_Template
DOP_GroupShellGranules::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template( PRM_INT_J,     1, &DOPactivationName, &DOPactivationDefault ),
    // Standard group parameter with group menu.
    PRM_Template( PRM_STRING,    1, &DOPgroupName, &DOPgroupDefault, &DOPgroupMenu ),
    // My added parameters
    PRM_Template( PRM_STRING,   1, &theInteriorGranulesGroupName, &defaultNull ),
    PRM_Template( PRM_STRING,   1, &theShellGranulesGroupName, &defaultNull ),
    PRM_Template( PRM_STRING,   1, &theMeshGranulesGroupName, &defaultNull ),
    PRM_Template( PRM_STRING,   1, &theShellGranulesForNewMeshGroupName, &defaultNull ),
    PRM_Template()
};

OP_Node *
DOP_GroupShellGranules::myConstructor(OP_Network *net, const char *name,
                                 OP_Operator *op)
{
    return new DOP_GroupShellGranules(net, name, op);
}

DOP_GroupShellGranules::DOP_GroupShellGranules(OP_Network *net, const char *name,
                                     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_GroupShellGranules::~DOP_GroupShellGranules()
{
}

void
DOP_GroupShellGranules::processObjectsSubclass(fpreal time, int,
                                          const SIM_ObjectArray &objects,
                                          DOP_Engine &engine)
{
    SIM_ObjectArray      filtered;
    UT_String            group;
    int                  i;         //, inputindex;

    // Grab the group string and filter our incoming objects using that
    // string. This narrows down the set of objects that we actually want
    // to operate on. The filtered array will contain only those objects
    // from the original objects array that match the supplied string.
    GROUP(group, time);
    SIM_DataFilterRootData       filter(group);
    objects.filter(filter, filtered);
    
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
        
    // Group all the shell granules that are not collided against mesh granules (mesh granules can also be interior granules).
    //   If they are not collided against mesh granules, they will be used to create new granule meshes.
    UT_String meshGranulesGroupName = "";
    MESHGRANULESGROUP( meshGranulesGroupName, time );
    const SIM_Relationship* meshGranulesGroup = NULL;
    if ( meshGranulesGroupName != "" )
        meshGranulesGroup = engine.getRelationship( meshGranulesGroupName );
    
    UT_String shellGranulesForNewMeshGroupName = "";
    SHELLGRANULESFORNEWMESHGROUPNAME( shellGranulesForNewMeshGroupName, time );
    
    //GB_AttributeRef objidAttrOffset = meshGdp->findPointAttrib( "objid", sizeof(int), GB_ATTRIB_INT );
	cout << "looking for shell granules" << endl;
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
            
            SIM_SnowNeighborData* currNeighborData = SIM_DATA_GET( *currObject, "Bullet Neighbor Data", SIM_SnowNeighborData );
            if ( currNeighborData )
            {
                bool isNewShellGranule = false;
                bool isShellGranule = false;
                
                // For each neighboring object to the current object, see if the neighbor is an interior granule
                int numNeighbors = currNeighborData->getNumNeighbors();
                for ( int n = 0; n < numNeighbors; n++ )
                {
                    // If the neighbor of currObject is an interior granule,
                    //   then add currObject to the shell granules group.
                    int neighborId = currNeighborData->getNeighborId( n );
                    SIM_Object* neighborObj = (SIM_Object*)engine.getSimulationObjectFromId( neighborId );
                    
                    if ( !neighborObj )
                        continue;
                    
                    if ( interiorGranulesGroup->getGroupHasObject( neighborObj ) )
                    {cout << "adding obj " << neighborObj->getName() << endl;
                        SIM_Relationship *shellGroup = engine.addRelationship( shellGranulesGroupName, SIM_DATA_RETURN_EXISTING );
                        if ( shellGroup && !isShellGranule)
                        {
                            shellGroup->addGroup( currObject );
                            SIM_DATA_CREATE( *shellGroup, SIM_RELGROUP_DATANAME,
                                            SIM_RelationshipGroup,
                                            SIM_DATA_RETURN_EXISTING );
                            isShellGranule = true;
                        }  // if
                        
                        isNewShellGranule = true;
                        
                        // If this is a shell granule and its neighbor is a mesh granule, then this shell granule will not be part of a new granular mesh
                        //   (it will join onto an old one).
                        if ( meshGranulesGroup && shellGranulesForNewMeshGroupName != "" )
                        {
                            if ( meshGranulesGroup->getGroupHasObject( neighborObj ) )
                            {
                                isNewShellGranule = false;
                                break;
                            }  // if
                        }  // if
                    }  // if
                }  // for n
                
                if ( isNewShellGranule )
                {cout << "  new shell granule " << currObject->getName() << endl;
                    SIM_Relationship *shellGranulesForNewMeshGroup = engine.addRelationship( shellGranulesForNewMeshGroupName, SIM_DATA_RETURN_EXISTING );
                    if ( shellGranulesForNewMeshGroup )
                    {
                        shellGranulesForNewMeshGroup->addGroup( currObject );
                        SIM_DATA_CREATE( *shellGranulesForNewMeshGroup, SIM_RELGROUP_DATANAME,
                                        SIM_RelationshipGroup,
                                        SIM_DATA_RETURN_EXISTING );
                    }  // if
                }  // if
            }  // if
        }  // if
    }  // for each object
}  // processObjectsSubclass()

void
DOP_GroupShellGranules::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info)
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
        info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
        info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_GroupShellGranules::getOutputInfoSubclass(int /*outputidx*/, DOP_InOutInfo &info)
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void
DOP_GroupShellGranules::GROUP(UT_String &str, fpreal t)
{
    evalString(str, DOPgroupName.getToken(), 0, t);
}

void DOP_GroupShellGranules::INTERIORGRANULESGROUPNAME( UT_String &str, float t )
{
    evalString( str, theInteriorGranulesGroupName.getToken(), 0, t );
}  // INTERIORGRANULESGROUPNAME

void DOP_GroupShellGranules::SHELLGRANULESGROUPNAME( UT_String &str, float t )
{
    evalString( str, theShellGranulesGroupName.getToken(), 0, t );
}  // SHELLGRANULESGROUPNAME

void DOP_GroupShellGranules::MESHGRANULESGROUP( UT_String &str, float t )
{
    evalString( str, theMeshGranulesGroupName.getToken(), 0, t );
}  // MESHGRANULESGROUP

void DOP_GroupShellGranules::SHELLGRANULESFORNEWMESHGROUPNAME( UT_String &str, float t )
{
    evalString( str, theShellGranulesForNewMeshGroupName.getToken(), 0, t );
}  // SHELLGRANULESFORNEWMESHGROUPNAME


