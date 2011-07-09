#include "DOP_GroupDeadMeshGranules.h"
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
    op = new DOP_Operator("hdk_groupdeadmeshgranules", "Group Dead Mesh Granules",
                          DOP_GroupDeadMeshGranules::myConstructor,
                          DOP_GroupDeadMeshGranules::myTemplateList, 1, 9999, 0,
                          0, 1);
    table->addOperator(op);
}




#define     POINT_CLASS     0
#define     PRIM_CLASS      1
#define     DETAIL_CLASS    2
#define     INT_TYPE        0
#define     FLOAT_TYPE      1
#define     STRING_TYPE     2

static PRM_Name         theNewGroupName( "newgroupname", "New Group Name" );
static PRM_Name         theMeshName( "meshname", "Mesh Name" );

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0, "" );
static PRM_Default      defaultObjid( 0, "objid" );

PRM_Template
DOP_GroupDeadMeshGranules::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template( PRM_INT_J,     1, &DOPactivationName, &DOPactivationDefault ),
    // Standard group parameter with group menu.
    PRM_Template( PRM_STRING,    1, &DOPgroupName, &DOPgroupDefault, &DOPgroupMenu ),
    // My added parameters
    PRM_Template( PRM_STRING,   1, &theNewGroupName, &defaultNull ),
    PRM_Template( PRM_STRING,   1, &theMeshName, &defaultNull ),
    PRM_Template()
};

OP_Node *
DOP_GroupDeadMeshGranules::myConstructor(OP_Network *net, const char *name,
                                 OP_Operator *op)
{
    return new DOP_GroupDeadMeshGranules(net, name, op);
}

DOP_GroupDeadMeshGranules::DOP_GroupDeadMeshGranules(OP_Network *net, const char *name,
                                     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_GroupDeadMeshGranules::~DOP_GroupDeadMeshGranules()
{
}

void
DOP_GroupDeadMeshGranules::processObjectsSubclass(fpreal time, int,
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
	
    // Get the new group name to create, based on the New Group Name parameter input
    UT_String newGroupName;
    NEWGROUPNAME( newGroupName, time );
    if ( newGroupName == "" )
    {
        cout << "ERROR: DOP_GroupDeadMeshGranules node has empty New Group parameter." << endl;
        return;
    }  // if
    
    // Get the mesh object, based on the Mesh Name parameter input
    UT_String meshName;
    MESHNAME( meshName, time );
    if ( newGroupName == "" )
    {
        cout << "ERROR: DOP_GroupDeadMeshGranules node has empty Mesh Name parameter." << endl;
        return;
    }  // if
    
    int nummatch;
    SIM_Object* meshObject = (SIM_Object*)engine.findObjectFromString( meshName, 0, &nummatch, time, 0 );
    if ( !meshObject )
        return;
	
	// The isActive function checks both the bypass flag and the
	// activation parameter on the node (if there is one, which there
	// is in this case). We call this function after calling
	// setCurrentObject and we call it for each object in case the
	// activation parameter uses some object-specific variables
	// like OBJID in an expression.
	
	SIM_Geometry *meshGeo = (SIM_Geometry *)meshObject->getGeometry();
	GU_DetailHandleAutoReadLock meshGdl( meshGeo->getGeometry() );
	GU_Detail *meshGdp = (GU_Detail *)meshGdl.getGdp();
	
	GB_AttributeRef objidAttrOffset = meshGdp->findPointAttrib( "objid", sizeof(int), GB_ATTRIB_INT );
	
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
		
		if( isActive(time) )
		{
            // Get the current object's objid attribute.
            SIM_Object* currObject = filtered(i);
            int objid = currObject->getObjectId();
            
            // Get the current object's creation time
            //  If this is the creation frame, don't add it to the deletion group
            fpreal creationTime = currObject->getCreationTime();
            //if ( creationTime == time )
             //   continue;
            
            // Compare the current dop object's objid with the objid of every point in the mesh.
            //   If a match is found, go on to the next dop object
            bool addCurrObjectToGroup = true;
            GEO_Point *pt;
            FOR_ALL_GPOINTS( meshGdp, pt )
            {
                int ptObjid = pt->getValue<int>( objidAttrOffset );
                if ( ptObjid == objid )
                {
                    addCurrObjectToGroup = false;
                    break;
                }  // if
                
            }  // FOR_ALL_GPOINTS
            
            // If NO matching point was found, add the current object to the group of dead mesh granules
            //   (i.e. mesh granules no longer having a matching vertex in the mesh).
            if ( addCurrObjectToGroup )
            {
                SIM_Relationship *relationship = engine.addRelationship( newGroupName, SIM_DATA_RETURN_EXISTING );
                if ( relationship )
                {
                    relationship->addGroup( currObject );
                    SIM_DATA_CREATE( *relationship, SIM_RELGROUP_DATANAME,
                                    SIM_RelationshipGroup,
                                    SIM_DATA_RETURN_EXISTING );
                }  // if
            }  // if
        }  // if
    }  // for each object
}  // processObjectsSubclass()

void
DOP_GroupDeadMeshGranules::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info)
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
        info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
        info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_GroupDeadMeshGranules::getOutputInfoSubclass(int /*outputidx*/, DOP_InOutInfo &info)
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void
DOP_GroupDeadMeshGranules::GROUP(UT_String &str, fpreal t)
{
    evalString(str, DOPgroupName.getToken(), 0, t);
}

void DOP_GroupDeadMeshGranules::NEWGROUPNAME( UT_String &str, float t )
{
    evalString( str, theNewGroupName.getToken(), 0, t );
}  // NEWGROUPNAME

void DOP_GroupDeadMeshGranules::MESHNAME( UT_String &str, float t )
{
    evalString( str, theMeshName.getToken(), 0, t );
}  // MESHNAME


