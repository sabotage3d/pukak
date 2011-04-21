#include "DOP_TransferObjidsToMesh.h"
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
    op = new DOP_Operator("hdk_transferobjidstomesh", "Transfer Objids to Mesh",
                          DOP_TransferObjidsToMesh::myConstructor,
                          DOP_TransferObjidsToMesh::myTemplateList, 1, 9999, 0,
                          0, 1);
    table->addOperator(op);
}




#define     POINT_CLASS     0
#define     PRIM_CLASS      1
#define     DETAIL_CLASS    2
#define     INT_TYPE        0
#define     FLOAT_TYPE      1
#define     STRING_TYPE     2

static PRM_Name         theObjectsGroup( "objgroup", "Objects Group" );
static PRM_Name         theMeshObjectName( "meshobjectname", "Mesh Object Name" );

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0, "" );
static PRM_Default      defaultObjid( 0, "objid" );

PRM_Template
DOP_TransferObjidsToMesh::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template( PRM_INT_J,     1, &DOPactivationName, &DOPactivationDefault ),
    // Standard group parameter with group menu.
    PRM_Template( PRM_STRING,    1, &theObjectsGroup, &DOPgroupDefault, &DOPgroupMenu ),
    // My added parameters
    PRM_Template( PRM_STRING,   1, &theMeshObjectName, &defaultNull ),
    PRM_Template()
};

OP_Node *
DOP_TransferObjidsToMesh::myConstructor(OP_Network *net, const char *name,
                                 OP_Operator *op)
{
    return new DOP_TransferObjidsToMesh(net, name, op);
}

DOP_TransferObjidsToMesh::DOP_TransferObjidsToMesh(OP_Network *net, const char *name,
                                     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_TransferObjidsToMesh::~DOP_TransferObjidsToMesh()
{
}

void
DOP_TransferObjidsToMesh::processObjectsSubclass(fpreal time, int,
                                          const SIM_ObjectArray &objects,
                                          DOP_Engine &engine)
{
    SIM_ObjectArray      filtered;
    UT_String            objectsGroup;
    int                  i;         //, inputindex;
    
    // Grab the group string and filter our incoming objects using that
    // string. This narrows down the set of objects that we actually want
    // to operate on. The filtered array will contain only those objects
    // from the original objects array that match the supplied string.
    OBJECTSGROUP( objectsGroup, time );
    //SIM_DataFilterRootData       filter(group);
    //objects.filter(filter, filtered);
    
    // Get the name of the interior granules group, based on the Interior Granules Group parameter input
    //   The interior granules are granules that are ready to be culled, whose outer neighbors we will set
    //   to be shell granules.
    UT_String meshObjectName;
    MESHOBJECTNAME( meshObjectName, time );
    //if ( meshObjectName == "" )
    //    return;
        
    // Get the mesh object
    //SIM_Object* meshObject = (SIM_Object*)engine.findObjectFromString( meshObjectName, 0, 0, time, false );
    SIM_Object* meshObject = objects(0);
    
    // Get the mesh geometry
    SIM_Geometry* meshGeo = (SIM_Geometry*)meshObject->getGeometry();
    GU_DetailHandleAutoReadLock meshGdl(meshGeo->getGeometry());
    GU_Detail *meshGdp = (GU_Detail *)meshGdl.getGdp();
    GB_AttributeRef objidAttrib = meshGdp->findPointAttrib( "objid", sizeof(int), GB_ATTRIB_INT );
    GB_AttributeRef permidAttrib = meshGdp->findPointAttrib( "permid", sizeof(int), GB_ATTRIB_INT );
    
    // Get the group of objects to transfer objids from
    const SIM_Relationship* objGroup = engine.getRelationship( objectsGroup );
    if ( !objGroup )
        return;
    
    int numObjs = objGroup->getGroupEntries();
    // Loop through all the objects that passed the filter.
    //for( i = 0; i < filtered.entries(); i++ )
    //{
    for ( i = 0; i < numObjs; i++ )
    {
        // Set information about the object we are going to process.
        // The first argument is the index of the current object within the
        // full list of objects we are going to process. The second
        // argument is the total number of objects we are going to process.
        // The last argument is a pointer to the actual object we are
        // processing.
        //setCurrentObject( i, filtered.entries(), filtered(i) );

        // The isActive function checks both the bypass flag and the
        // activation parameter on the node (if there is one, which there
        // is in this case). We call this function after calling
        // setCurrentObject and we call it for each object in case the
        // activation parameter uses some object-specific variables
        // like OBJID in an expression.
        if( isActive(time) )
        {
            //SIM_Object* currObject = filtered(i);
            SIM_Object* currObject = (SIM_Object*)objGroup->getGroupObject( i );
            
            // Get the mesh geometry
            SIM_Geometry* currGeo = (SIM_Geometry*)currObject->getGeometry();
            GU_DetailHandleAutoReadLock     currGdl(currGeo->getGeometry());
            GU_Detail *currGdp = (GU_Detail *)currGdl.getGdp();
            
            // Get the current object's objid attribute.
            int objid = currObject->getObjectId();
            
            // Get the current object's permid attribute.
            SIM_EmptyData* copyInfo = SIM_DATA_GET( *currObject, "CopyInfo", SIM_EmptyData );
            if ( !copyInfo )
            {
                cout << "WARNING: DOP_TransferObjidsToMesh: No CopyInfo available" << endl;
                return;
            }  // if
            
            int objPermid = copyInfo->getData().getOptionF( "permid" );
            GEO_Point *pt;
            FOR_ALL_GPOINTS( meshGdp, pt )
            {
                int ptPermid = pt->getValue<int>( permidAttrib );
                if ( ptPermid == objPermid )
                {
                    pt->setValue<int>( objidAttrib, objid );
                    break;
                }  // if
            }  // FOR_ALL_GPOINTS
        }  // if
    }  // for each object
}  // processObjectsSubclass()

void
DOP_TransferObjidsToMesh::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info)
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
        info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
        info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_TransferObjidsToMesh::getOutputInfoSubclass(int /*outputidx*/, DOP_InOutInfo &info)
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void
DOP_TransferObjidsToMesh::OBJECTSGROUP(UT_String &str, fpreal t)
{
    evalString(str, theObjectsGroup.getToken(), 0, t);
}

void DOP_TransferObjidsToMesh::MESHOBJECTNAME( UT_String &str, float t )
{
    evalString( str, theMeshObjectName.getToken(), 0, t );
}  // INTERIORGRANULESGROUPNAME
