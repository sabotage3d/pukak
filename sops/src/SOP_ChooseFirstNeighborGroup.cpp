#include "SOP_ChooseFirstNeighborGroup.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
#include <DOP/DOP_Engine.h>
#include <DOP/DOP_Parent.h>
//#include <GB/GB_EdgeGroup.h>
#include <GA/GA_EdgeGroup.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimSphere.h>
#include <PRM/PRM_Default.h>
#include <PRM/PRM_Include.h>
#include <OBJ/OBJ_DopNet.h>
#include <OBJ/OBJ_Node.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_Relationship.h>
#include <SOP/SOP_Guide.h>
#include <UT/UT_Vector4.h>
#include <UT/UT_Vector4Array.h>

#include <stdlib.h>
#include <iostream>
using namespace std;




static PRM_Name        names[] = {
	PRM_Name("neighborgroupsmask", "Neighbor Groups Mask"),
    PRM_Name("meshinteriorgroup", "Mesh Interior Group"),
    PRM_Name("interiorgroup", "Interior Group"),
	PRM_Name("chosengroupname", "Chosen Group Name"),
};

static PRM_Default          defEmptyString( 0, "" );

PRM_Template
SOP_ChooseFirstNeighborGroup::myTemplateList[] = {
	PRM_Template( PRM_STRING, 1, &names[0], &defEmptyString, 0, 0, 0, 0, 1, "Mask for the neighbor groups' prefix.  It is a list of groups to choose from, each group having one interior point and its neighboring (colliding) points." ),
    PRM_Template( PRM_STRING, 1, &names[1], &defEmptyString, 0, 0, 0, 0, 1, "Group containing a subset of interior points, those attached to the mesh, from which to choose first." ),
    PRM_Template( PRM_STRING, 1, &names[2], &defEmptyString, 0, 0, 0, 0, 1, "Group containing all the interior points." ),
	PRM_Template( PRM_STRING, 1, &names[3], &defEmptyString, 0, 0, 0, 0, 1, "Group to put the chose group's points into." ),
    PRM_Template(),
};




void
newSopOperator( OP_OperatorTable *table )
{
     table->addOperator(
                    new OP_Operator("choosefirstneighborgroup",
					"Choose First Neighbor Group",
					SOP_ChooseFirstNeighborGroup::myConstructor,
					SOP_ChooseFirstNeighborGroup::myTemplateList,
					1,
					1,
					0)
                        );
}




OP_Node *
SOP_ChooseFirstNeighborGroup::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_ChooseFirstNeighborGroup(net, name, op);
}




SOP_ChooseFirstNeighborGroup::SOP_ChooseFirstNeighborGroup( OP_Network *net, const char *name, OP_Operator *op )
	: SOP_Node(net, name, op), myGroup(0)
{
}

SOP_ChooseFirstNeighborGroup::~SOP_ChooseFirstNeighborGroup() {}




// Main cooking method.
//   This takes in two input geometries and the name of an attribute they both share.
//   All points in the first input geometry that have a matching value for that
//   attribute in the second input geometry are grouped into a new group.
OP_ERROR SOP_ChooseFirstNeighborGroup::cookMySop( OP_Context &context )
{
    // Before we do anything, we must lock our inputs.  Before returning,
    //	we have to make sure that the inputs get unlocked.
    if ( lockInputs(context) >= UT_ERROR_ABORT )
        return error();
    
    this->flags().setTimeDep(1);
    
    float t = context.myTime;
    
    // Duplicate our incoming geometry with the hint that we only
    // altered points.  Thus if we our input was unchanged we can
    // easily roll back our changes by copying point values.
    duplicatePointSource(0, context);
    
    if ( error() < UT_ERROR_ABORT )
    {
        
        // Get the name of the interior primitive group
		UT_String neighborGroupsMask = NEIGHBORGROUPSMASK(t);
        UT_String meshInteriorGroupName = MESHINTERIORGROUP(t);		// Name of the attribute to find matches with between the two input geometries
		UT_String interiorGroupName = INTERIORGROUP(t);
		UT_String chosenGroupName = CHOSENGROUPNAME(t);
		
		neighborGroupsMask.strip("*");
		
		// Get the attribute
		//GB_AttributeRef objidAttrRef = gdp->findPointAttrib( "objid", GB_ATTRIB_INT );
		GA_RWAttributeRef objidAttrRef = gdp->findIntTuple( GA_ATTRIB_POINT, "objid", 1 );   // Tuple size = 1
		if ( objidAttrRef.isInvalid() )
        {
			addError( SOP_ATTRIBUTE_INVALID, "There is no objid attribute in your geometry." );
		}  // if
		
		// Make the new group to put the chosen neighbor group into
		GA_PointGroup* chosenGroup = gdp->newPointGroup( chosenGroupName );
		
		// Get the meshInteriorGroup of points
		const GA_PointGroup *meshInteriorGroup = parsePointGroups( (const char*)meshInteriorGroupName, gdp );
		if ( !meshInteriorGroup )
		{
			addError( SOP_ERR_BADGROUP, meshInteriorGroupName );
			return error();
		}  // if
		if ( meshInteriorGroup->entries() > 0 )		// If there are interior granules in the mesh
		{
			// Get the objid of the FIRST point in the mesh interior group
			GEO_Point* interiorPt;
			int objid = -1;
			GA_FOR_ALL_GROUP_POINTS( gdp, meshInteriorGroup, interiorPt )
			{
				objid = interiorPt->getValue<int>( objidAttrRef );
				break;
			}  // FOR_ALL_GROUP_POINTS
			
			char tmp[181];
            sprintf( tmp, "%s%d", (char*)neighborGroupsMask, objid );
            UT_String neighborGroupName = tmp;
			
			// Get the neighbor point group
			GA_PointGroup* neighborPointGroup = (GA_PointGroup*)parsePointGroups( (const char*)neighborGroupName, gdp );
			
			// Copy the neighbor group points into the new group
			GEO_Point* pt;
			GA_FOR_ALL_OPT_GROUP_POINTS( gdp, neighborPointGroup, pt )
			{
				chosenGroup->add( pt );
			}  // FOR_ALL_GPOINTS
			
			// Destroy the old point group
			gdp->destroyPointGroup( neighborPointGroup );
		}  // if
		else			// Get the list of ALL the interior granules
		{
			const GA_PointGroup *interiorGroup = parsePointGroups( (const char*)interiorGroupName, gdp );
			if ( !interiorGroup )
			{
				addError( SOP_ERR_BADGROUP, interiorGroupName );
				return error();
			}  // if
			if ( interiorGroup->entries() > 0 )
			{
				// Get the objid of the FIRST point in the interior group
				GEO_Point* interiorPt;
				int objid = -1;
				GA_FOR_ALL_GROUP_POINTS( gdp, interiorGroup, interiorPt )
				{
					objid = interiorPt->getValue<int>( objidAttrRef );
					break;
				}  // FOR_ALL_GROUP_POINTS
				
				char tmp[181];
				sprintf( tmp, "%s%d", (char*)neighborGroupsMask, objid );
				UT_String neighborGroupName = tmp;
				
				// Get the neighbor point group
				GA_PointGroup *neighborPointGroup = (GA_PointGroup*)parsePointGroups( (const char*)neighborGroupName, gdp );
				
				// Copy the neighbor group points into the new group
				GEO_Point* pt;
				GA_FOR_ALL_OPT_GROUP_POINTS( gdp, neighborPointGroup, pt )
				{
					chosenGroup->add( pt );
				}  // FOR_ALL_GPOINTS
				
				// Destroy the old point group
				gdp->destroyPointGroup( neighborPointGroup );
			}  // if
		}  // else
    }  // if
    
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    //gdp->notifyCache(GU_CACHE_ALL);
    
    return error();
}

