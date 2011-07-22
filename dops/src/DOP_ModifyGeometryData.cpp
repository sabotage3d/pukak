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




#define     POINT_CLASS     0
#define     PRIM_CLASS      1
#define     DETAIL_CLASS    2
#define     INT_TYPE        0
#define     FLOAT_TYPE      1
#define     STRING_TYPE     2

//static PRM_Name         theInputIndexName("inputindex", "Input Index");
static PRM_Name         theValueNumber( "valuenumber", "Value Number" );
static PRM_Name         theValueString( "valuestring", "Value String" );
static PRM_Name         theGeomAttrName( "geomattrname", "Geom Attr Name" );
static PRM_Name         theGeomAttrClass( "geomattrclass", "Geom Attr Class" );
static PRM_Name         theGeomAttrType( "geomattrtype", "Geom Attr Type" );
static PRM_Name        theGeomAttrClassNames[] = {
       PRM_Name( "pointclass",     "Point" ),
       PRM_Name( "primclass",      "Prim" ),
       PRM_Name( "detailclass",    "Detail" ),
       PRM_Name(0)
};
static PRM_Name        theGeomAttrTypeNames[] = {
       PRM_Name( "inttype",        "Integer" ),
       PRM_Name( "floattype",      "Float" ),
       PRM_Name( "stringtype",     "String" ),
       PRM_Name(0)
};

static PRM_ChoiceList   theGeomAttrClassesMenu((PRM_ChoiceListType)(PRM_CHOICELIST_REPLACE | PRM_CHOICELIST_EXCLUSIVE ), &(theGeomAttrClassNames[0]) );
static PRM_ChoiceList   theGeomAttrTypesMenu((PRM_ChoiceListType)(PRM_CHOICELIST_REPLACE | PRM_CHOICELIST_EXCLUSIVE ), &(theGeomAttrTypeNames[0]) );

static PRM_Default      defaultNull( 0, "" );
static PRM_Default      defaultZero( 0, "" );
static PRM_Default      defaultObjid( 0, "objid" );

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
    //PRM_Template( PRM_INT_J,    1, &theInputIndexName, &defaultNull),
    
    PRM_Template( PRM_FLT_J,   1, &theValueNumber, &defaultZero ),
    PRM_Template( PRM_STRING,   1, &theValueString, &defaultNull ),
    PRM_Template( PRM_STRING,   1, &theGeomAttrName, &defaultObjid ),        // The name template is a string, has 1 channel, is name attrname, and defaults to empty string
    PRM_Template( PRM_INT_J,    1, &theGeomAttrClass, &defaultZero, &theGeomAttrClassesMenu ),
    PRM_Template( PRM_INT_J,    1, &theGeomAttrType, &defaultZero, &theGeomAttrTypesMenu ),
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
    int                  i;         //, inputindex;

    // Grab the group string and filter our incoming objects using that
    // string. This narrows down the set of objects that we actually want
    // to operate on. The filtered array will contain only those objects
    // from the original objects array that match the supplied string.
    GROUP(group, time);
    SIM_DataFilterRootData       filter(group);
    objects.filter(filter, filtered);
	
	//for( i = 0; i < objects.entries(); i++ )
	//	cout << objects(i)->getName() << endl;

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
            //SIM_Geometry* geom = (SIM_Geometry *)currObject->getGeometry();
            int objid = currObject->getObjectId();
			
			SIM_GeometryCopy *copygeo = SIM_DATA_GET( *currObject, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy );
			if ( !copygeo )
			{
				copygeo = SIM_DATA_CREATE( *currObject, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy, SIM_DATA_ADOPT_EXISTING_ON_DELETE );
			}  // if
            
            if( copygeo )
            {
                //GU_DetailHandleAutoReadLock gdl( geom->getGeometry() );
                //GU_Detail *gdp = (GU_Detail *)gdl.getGdp();
                
                UT_String geomAttrName;
                int geomAttrClass;
                int geomAttrType;
                GEOMATTRNAME( geomAttrName, time );     // Gets a string
                geomAttrClass = GEOMATTRCLASS( time );  // Gets a number
                geomAttrType = GEOMATTRTYPE( time );    // Gets a number
                
                //SIM_GeometryCopy *copygeo = 0;
                //copygeo = (SIM_GeometryCopy*)( SIM_DATA_GET( *currObject, SIM_GEOMETRY_DATANAME, SIM_Geometry ) );
				
				//copygeo = SIM_DATA_GET( *currObject, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy );
				//if ( !copygeo )
				//{
				//	cout << "creating new geom" << endl;
				//	copygeo = SIM_DATA_CREATE( *currObject, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy, 0 );
				//}  // if
                //if ( !copygeo )
                //{
				//	cout << "no copy geo found" << endl;
                //    continue;
                //}
				
                //GU_DetailHandleAutoReadLock gdl = GU_DetailHandleAutoReadLock( copygeo->getGeometry() );
				GU_DetailHandleAutoWriteLock gdl( copygeo->lockGeometry() );
                GU_Detail* gdp = gdl.getGdp();
                
                if ( geomAttrClass == POINT_CLASS )
                {
                    GEO_Point *pt;
                    if ( geomAttrType == INT_TYPE )
                    {
                        //GB_AttributeRef attRef = gdp->findPointAttrib( geomAttrName, GB_ATTRIB_INT );
						//GA_WOAttributeRef attRef = gdp->findIntTuple( GA_ATTRIB_POINT, GA_SCOPE_PUBLIC, geomAttrName );
						GA_RWAttributeRef attRef = gdp->findIntTuple( GA_ATTRIB_POINT, geomAttrName, 1 );	// Tuple size is 1
                        
                        if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPointAttrib( geomAttrName, sizeof(int), GB_ATTRIB_INT, 0 );
							attRef = gdp->addIntTuple( GA_ATTRIB_POINT, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
                        
                        if ( attRef.isValid() )
                        {
                            int valueNumber = (int)VALUENUMBER( time );
                            GA_FOR_ALL_GPOINTS( gdp, pt )
                            {
                                pt->setValue( attRef, valueNumber );
                            }  // for each point in gd
                        }  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else
                    }  // if
                    else if ( geomAttrType == FLOAT_TYPE )
                    {
                        //GB_AttributeRef attRef = gdp->findPointAttrib( geomAttrName, GB_ATTRIB_FLOAT );
						GA_RWAttributeRef attRef = gdp->findFloatTuple( GA_ATTRIB_POINT, geomAttrName, 1 );	// tuple size = 1
                        if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPointAttrib( geomAttrName, sizeof(float), GB_ATTRIB_FLOAT, 0 );
							attRef = gdp->addFloatTuple( GA_ATTRIB_POINT, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
                        
                        if ( attRef.isValid() )
                        {
                            float valueNumber = VALUENUMBER( time );
                            GA_FOR_ALL_GPOINTS( gdp, pt )
                            {
                                pt->setValue( attRef, valueNumber );
                            }  // for each point in gdp
                        }  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else
                    }  // else if
                    else if ( geomAttrType == STRING_TYPE )
                    {
                        //cerr << "hdk_modifygeomdata: String type not allowed yet." << endl;
						//GB_AttributeRef attRef = gdp->findPointAttrib( geomAttrName, sizeof(int), GB_ATTRIB_INDEX );
						//GEO_AttributeHandle attrHandle = gdp->getPointAttribute( geomAttrName );
						GA_RWAttributeRef attRef = gdp->findStringTuple( GA_ATTRIB_POINT, geomAttrName, 1 );	// tuple size = 1
						//GEO_AttributeHandle attrHandle = gdp->getPointAttribute( geomAttrName );
						if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPointAttrib( geomAttrName, sizeof(float), GB_ATTRIB_FLOAT, 0 );
							attRef = gdp->addStringTuple( GA_ATTRIB_POINT, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
						
						if ( attRef.isValid() )
                        {
                            UT_String valueString;
							VALUESTRING( valueString, time );
							
							//GB_Attribute* attr = gdp->pointAttribs().find( geomAttrName, sizeof(int), GB_ATTRIB_INDEX);
							//int strIndex = attr->addIndex( valueString );
							GA_FOR_ALL_GPOINTS( gdp, pt )
							{
								pt->setString( attRef, valueString );
							}  // for each point in gd
						}
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else
						
                    }  // else if
                    else
                    {
                        addError( DOP_INVALIDINPUT, "Bad Attribute Type" );
                    }  // else
                }  // if
                else if ( geomAttrClass == PRIM_CLASS )
                {
                    GEO_Primitive *prim;
                    if ( geomAttrType == INT_TYPE )
                    {
                        //GB_AttributeRef attRef = gdp->findPrimAttrib( geomAttrName, 0, GB_ATTRIB_INT );
						GA_RWAttributeRef attRef = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, geomAttrName, 1 );	// Tuple size is 1
                        if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPrimAttrib( geomAttrName, sizeof(int), GB_ATTRIB_INT, 0 );
							attRef = gdp->addIntTuple( GA_ATTRIB_PRIMITIVE, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
                        
                        if ( attRef.isValid() )
                        {
                            int valueNumber = (int)VALUENUMBER( time );
                            GA_FOR_ALL_PRIMITIVES( gdp, prim )
                            {
                                prim->setValue<int>( attRef, valueNumber );
                            }  // for each point in gd
                        }  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else
                    }  // if
                    else if ( geomAttrType == FLOAT_TYPE )
                    {
                        //GB_AttributeRef attRef = gdp->findPrimAttrib( geomAttrName, 0, GB_ATTRIB_FLOAT );
						GA_RWAttributeRef attRef = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, geomAttrName, 1 );	// Tuple size is 1
                        if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPrimAttrib( geomAttrName, sizeof(float), GB_ATTRIB_FLOAT, 0 );
							attRef = gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
                        
                        if ( attRef.isValid() )
                        {
                            float valueNumber = VALUENUMBER( time );
                            GA_FOR_ALL_PRIMITIVES( gdp, prim )
                            {
                                prim->setValue<float>( attRef, valueNumber );
                            }  // for each point in gd
                        }  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else
                    }  // else if
                    else if ( geomAttrType == STRING_TYPE )
                    {
                        GA_RWAttributeRef attRef = gdp->findStringTuple( GA_ATTRIB_PRIMITIVE, geomAttrName, 1 );	// tuple size = 1
						if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPointAttrib( geomAttrName, sizeof(float), GB_ATTRIB_FLOAT, 0 );
							attRef = gdp->addStringTuple( GA_ATTRIB_PRIMITIVE, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
						
						UT_String valueString;
                        VALUESTRING( valueString, time );
						
						//GB_Attribute* attr = gdp->pointAttribs().find( geomAttrName, sizeof(int), GB_ATTRIB_INDEX);
						//int strIndex = attr->addIndex( valueString );
						GA_FOR_ALL_PRIMITIVES( gdp, prim )
						{
							prim->setString( attRef, valueString );
						}  // for each point in gd
					}
                    else
                    {
                        addError( DOP_INVALIDINPUT, "Bad Attribute Type" );
                    }  // else
                }  // else if
                else if ( geomAttrClass == DETAIL_CLASS )
                {
                    if ( geomAttrType == INT_TYPE )
                    {
                        //GB_AttributeRef attRef = gdp->findAttrib( geomAttrName, GB_ATTRIB_INT );
						/*int valueNumber = (int)VALUENUMBER( time );
						
						GB_AttributeRef attrOffset = gdp->attribs().getOffset( geomAttrName, GB_ATTRIB_INT );
                        if ( attrOffset.isInvalid() )
                        {
                            // Create the point attribute
                            attrOffset = gdp->addAttrib( geomAttrName, sizeof(int), GB_ATTRIB_INT, 0 );
                        }  // if
                        
						if ( attrOffset.isValid() )
                        {
							//gdp->attribs().getElement().setValue<int>( attrOffset, valueNumber );
							gdp->setDetailAttributeI( geomAttrName, valueNumber );
						}  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else*/
						GA_RWAttributeRef attRef = gdp->findIntTuple( GA_ATTRIB_DETAIL, geomAttrName, 1 );	// Tuple size is 1
                        if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPointAttrib( geomAttrName, sizeof(int), GB_ATTRIB_INT, 0 );
							attRef = gdp->addIntTuple( GA_ATTRIB_DETAIL, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
                        
                        if ( attRef.isValid() )
                        {
                            int valueNumber = (int)VALUENUMBER( time );
                            //GA_FOR_ALL_GPOINTS( gdp, pt )
                            //{
                            //    pt->setValue<int>( attRef, valueNumber );
                            //}  // for each point in gd
							gdp->element().setValue<int>( attRef, valueNumber );
                        }  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else
                    }  // if
                    else if ( geomAttrType == FLOAT_TYPE )
                    {
                        //GB_AttributeRef attRef = gdp->findAttrib( geomAttrName, GB_ATTRIB_FLOAT );
						/*int valueNumber = VALUENUMBER( time );
						
						GB_AttributeRef attrOffset = gdp->attribs().getOffset( geomAttrName, GB_ATTRIB_FLOAT );
                        if ( attrOffset.isInvalid() )
                        {
                            // Create the point attribute
                            attrOffset = gdp->addAttrib( geomAttrName, sizeof(float), GB_ATTRIB_FLOAT, 0 );
                        }  // if
                        
                        if ( attrOffset.isValid() )
                        {
							gdp->setDetailAttributeF( geomAttrName, valueNumber );
                        }  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else*/
						
						GA_RWAttributeRef attRef = gdp->findFloatTuple( GA_ATTRIB_DETAIL, geomAttrName, 1 );	// Tuple size is 1
                        if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPointAttrib( geomAttrName, sizeof(int), GB_ATTRIB_INT, 0 );
							attRef = gdp->addFloatTuple( GA_ATTRIB_DETAIL, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
                        
                        if ( attRef.isValid() )
                        {
                            float valueNumber = (float)VALUENUMBER( time );
                            //GA_FOR_ALL_GPOINTS( gdp, pt )
                            //{
                            //    pt->setValue<int>( attRef, valueNumber );
                            //}  // for each point in gd
							gdp->element().setValue<float>( attRef, valueNumber );
                        }  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else
                    }  // else if
                    else if ( geomAttrType == STRING_TYPE )
                    {
                        //cerr << "hdk_modifygeomdata: String type not allowed yet." << endl;
						GA_RWAttributeRef attRef = gdp->findStringTuple( GA_ATTRIB_DETAIL, geomAttrName, 1 );	// tuple size = 1
						if ( attRef.isInvalid() )
                        {
                            // Create the point attribute
                            //attRef = gdp->addPointAttrib( geomAttrName, sizeof(float), GB_ATTRIB_FLOAT, 0 );
							attRef = gdp->addStringTuple( GA_ATTRIB_DETAIL, geomAttrName, 1 );	// Tuple size is 1
                        }  // if
						
						if ( attRef.isValid() )
                        {
							UT_String valueString;
							VALUESTRING( valueString, time );
							gdp->element().setString( attRef, valueString );
                        }  // if
                        else
                        {
                            addError( DOP_NODATA, geomAttrName );
                        }  // else
					}
                    else
                    {
                        addError( DOP_INVALIDINPUT, "Bad Attribute Type" );
                    }  // else
                }
                else
                {
                    addError( DOP_INVALIDINPUT, "Bad Attribute Class" );
                }
                
                copygeo->releaseGeometry();
            }  // if
            else
            {
                addError( DOP_NODATA, "Geometry" );
            }
        }  // if
    }  // for each object
}  // processObjectsSubclass()

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

//int
//DOP_ModifyGeometryData::INPUTINDEX(float t)
//{
//    return evalInt( theInputIndexName.getToken(), 0, t );
//}  // INPUTINDEX

float DOP_ModifyGeometryData::VALUENUMBER( float t )
{
    return evalInt( theValueNumber.getToken(), 0, t );
}  // VALUENUMBER

void DOP_ModifyGeometryData::VALUESTRING( UT_String &str, float t )
{
    evalString( str, theValueString.getToken(), 0, t );
}  // VALUESTRING

void DOP_ModifyGeometryData::GEOMATTRNAME( UT_String& str, float t )
{
    evalString( str, theGeomAttrName.getToken(), 0, t );
}  // GEOMATTRNAME

int DOP_ModifyGeometryData::GEOMATTRCLASS( float t )
{
    return evalInt( theGeomAttrClass.getToken(), 0, t );
}  // GEOMATTRCLASS

int DOP_ModifyGeometryData::GEOMATTRTYPE( float t )
{
    return evalInt( theGeomAttrType.getToken(), 0, t );
}  // GEOMATTRTYPE


