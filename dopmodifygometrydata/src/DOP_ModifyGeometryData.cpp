#include "DOP_ModifyGeometryData.h"
#include <UT/UT_DSOVersion.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipGroup.h>
#include <OP/OP_OperatorTable.h>
#include <DOP/DOP_PRMShared.h>
#include <DOP/DOP_InOutInfo.h>
#include <DOP/DOP_Operator.h>
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
    op = new DOP_Operator("hdk_modifygeometrydata", "Modify Geom Data",
                          DOP_ModifyGeometryData::myConstructor,
                          DOP_ModifyGeometryData::myTemplateList, 1, 9999, 0,
                          0, 1);
    table->addOperator(op);
}

static PRM_Name          theInputIndexName("inputindex", "Input Index");

PRM_Template
DOP_ModifyGeometryData::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template(PRM_INT_J,     1, &DOPactivationName,
                                &DOPactivationDefault),
    // Standard group parameter with group menu.
    PRM_Template(PRM_STRING,    1, &DOPgroupName, &DOPgroupDefault,
                                &DOPgroupMenu),
    // The input index that determines which data will be attached to
    // each object.
    PRM_Template(PRM_INT_J,     1, &theInputIndexName, PRMzeroDefaults),
    PRM_Template()
};

OP_Node *
DOP_ModifyGeometryData::myConstructor(OP_Network *net, const char *name,
                                 OP_Operator *op)
{
    return new DOP_ModifyGeometryData(net, name, op);
}

DOP_ModifyGeometryData::DOP_ModifyGeometryData(OP_Network *net, const char *name,
                                     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_ModifyGeometryData::~DOP_ModifyGeometryData()
{
}

void
DOP_ModifyGeometryData::processObjectsSubclass(fpreal time, int,
                                          const SIM_ObjectArray &objects,
                                          DOP_Engine &engine)
{
    SIM_ObjectArray      filtered;
    UT_String            group;
    int                  i, inputindex;

    // Grab the group string and filter our incoming objects using that
    // string. This narrows down the set of objects that we actually want
    // to operate on. The filtered array will contain only those objects
    // from the original objects array that match the supplied string.
    GROUP(group, time);
    SIM_DataFilterRootData       filter(group);
    objects.filter(filter, filtered);

    // Loop through all the objects that passed the filter.
    for( i = 0; i < filtered.entries(); i++ )
    {
        // Set information about the object we are going to process.
        // The first argument is the index of the current object within the
        // full list of objects we are going to process. The second
        // argument is the total number of objects we are going to process.
        // The last argument is a pointer to the actual object we are
        // processing.
        setCurrentObject(i, filtered.entries(), filtered(i));

        // The isActive function checks both the bypass flag and the
        // activation parameter on the node (if there is one, which there
        // is in this case). We call this function after calling
        // setCurrentObject and we call it for each object in case the
        // activation parameter uses some object-specific variables
        // like OBJID in an expression.
        if( isActive(time) )
        {
            // Evaluate the input index. Also called after setCurrentObject
            // to properly evaluate objects specific local variables. Then
            // make sure there is something connected to the requested input.
            // We add one to the returned value to skip over the object
            // input.
            //inputindex = INPUTINDEX(time) + 1;
            //if( inputindex != 0 && getInput(inputindex) )
            //{
                SIM_Object *currObject = filtered(i);
                SIM_Geometry* geom = (SIM_Geometry *)currObject->getGeometry();
                int objid = currObject->getObjectId();
                
                if( geom )
                {
                    //GU_DetailHandleAutoReadLock gdl( geom->getGeometry() );
                    //GU_Detail *gdp = (GU_Detail *)gdl.getGdp();
                    
                    
                    SIM_GeometryCopy *copygeo = 0;
                    copygeo = (SIM_GeometryCopy*)( SIM_DATA_GET( *currObject, SIM_GEOMETRY_DATANAME, SIM_Geometry ) );
                    if ( !copygeo )
                    {
                        continue;
                    }
                    GU_DetailHandleAutoReadLock gdl = GU_DetailHandleAutoReadLock( copygeo->getGeometry() );
                    GU_Detail* gdp = (GU_Detail *)gdl.getGdp();
                    GB_AttributeRef attObjid = gdp->findPointAttrib( "objid", GB_ATTRIB_INT );
                    
                    
                    GEO_Point *pt;
                    FOR_ALL_GPOINTS( gdp, pt )
                    {
                        if ( attObjid.isValid() )
                        {
                            pt->setValue<int>( attObjid, objid );
                        }  // if
                        else
                        {
                            cerr << "hdk_modifygeometrydata: Bad attribute." << endl;
                            cerr << "  Make sure you've attached the attribute to the geometry and that it is the right type." << endl;
                        }
                    }  // for each point in gdp
                }  // if
                else
                {
                    addError( DOP_NODATA, "Geometry" );
                }
                
                /*SIM_Relationship        *relationship;
                UT_String                addtogroup;
                char                     numstr[UT_NUMBUF];

                // This function attaches the data connected to input
                // number inputindex to the current object.
                applyDataFromInput(time, inputindex, inputindex,
                                   *filtered(i),
                                   0, engine, 0, true);
                // Create a group name based on the input index.
                UT_String::itoa(numstr, inputindex);
                addtogroup = "applygroup_";
                addtogroup += numstr;

                // Create the relationship if it doesn't already exist,
                // and add the object to the group.
                relationship = engine.addRelationship(addtogroup,
                                      SIM_DATA_RETURN_EXISTING);
                if( relationship )
                {
                    relationship->addGroup(filtered(i));
                    SIM_DATA_CREATE(*relationship, SIM_RELGROUP_DATANAME,
                                    SIM_RelationshipGroup,
                                    SIM_DATA_RETURN_EXISTING);
                }
                else
                    addError(DOP_CANTCREATERELATIONSHIP);
                */
            //}
        }
    }
}

void
DOP_ModifyGeometryData::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info)
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
        info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
        info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_ModifyGeometryData::getOutputInfoSubclass(int /*outputidx*/, DOP_InOutInfo &info)
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void
DOP_ModifyGeometryData::GROUP(UT_String &str, fpreal t)
{
    evalString(str, DOPgroupName.getToken(), 0, t);
}

int
DOP_ModifyGeometryData::INPUTINDEX(float t)
{
    return evalInt(theInputIndexName.getToken(), 0, t);
}


