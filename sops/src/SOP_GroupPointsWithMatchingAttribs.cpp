#include "SOP_GroupPointsWithMatchingAttribs.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
#include <DOP/DOP_Engine.h>
#include <DOP/DOP_Parent.h>
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
	PRM_Name("group", "Group"),
    PRM_Name("attribname", "Attribute Name"),
    PRM_Name("matchingpointsgroup", "Matching Pts Group"),
};

static PRM_Default          defEmptyString( 0, "" );

PRM_Template
SOP_GroupPointsWithMatchingAttribs::myTemplateList[] = {
	PRM_Template( PRM_STRING, 1, &names[0], &defEmptyString, 0, 0, 0, 0, 1, "Group of points to search for matches." ),
    PRM_Template( PRM_STRING, 1, &names[1], &defEmptyString, 0, 0, 0, 0, 1, "Name of the point attribute to find matching values between the points of the two input geometries." ),
    PRM_Template( PRM_STRING, 1, &names[2], &defEmptyString, 0, 0, 0, 0, 1, "Name of the group to create and add points that have matching points (for the given attribute) in the second input geometry." ),
    PRM_Template(),
};




void
newSopOperator( OP_OperatorTable *table )
{
     table->addOperator(
                    new OP_Operator("grouppointswithmatchingattribs",
					"Group Points With Matching Attributes",
					SOP_GroupPointsWithMatchingAttribs::myConstructor,
					SOP_GroupPointsWithMatchingAttribs::myTemplateList,
					2,
					2,
					0)
                        );
}




OP_Node *
SOP_GroupPointsWithMatchingAttribs::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_GroupPointsWithMatchingAttribs(net, name, op);
}




SOP_GroupPointsWithMatchingAttribs::SOP_GroupPointsWithMatchingAttribs( OP_Network *net, const char *name, OP_Operator *op )
	: SOP_Node(net, name, op), myGroup(0)
{
}

SOP_GroupPointsWithMatchingAttribs::~SOP_GroupPointsWithMatchingAttribs() {}




// Main cooking method.
//   This takes in two input geometries and the name of an attribute they both share.
//   All points in the first input geometry that have a matching value for that
//   attribute in the second input geometry are grouped into a new group.
OP_ERROR SOP_GroupPointsWithMatchingAttribs::cookMySop( OP_Context &context )
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
		UT_String groupName = GROUP(t);
        UT_String attribName = ATTRIBNAME(t);		// Name of the attribute to find matches with between the two input geometries
		UT_String matchingPointsGroupName = MATCHINGPOINTSGROUP(t);
		
		// Get the group of particles to act on
		const GA_PointGroup *group = NULL;
		if ( groupName.isstring())
		{
			group = parsePointGroups( (const char *)groupName, gdp );
			if ( !group )
			{
				addError( SOP_ERR_BADGROUP, groupName );
			}  // if
		}  // if
		
		// Get the second input geometry.
		//   (We already have the first input geometry, gdp).
		const GU_Detail* gdp2 = inputGeo( 1, context );
		
		// Get the attributes for each input geometry
		//GB_AttributeRef attRef1 = gdp->findPointAttrib( attribName, GB_ATTRIB_INT );
		GA_ROAttributeRef attRef1 = gdp->findIntTuple( GA_ATTRIB_POINT, attribName, 1 );		// tuple of size 1
		//GB_AttributeRef attRef2 = gdp2->findPointAttrib( attribName, GB_ATTRIB_INT );
		GA_ROAttributeRef attRef2 = gdp2->findIntTuple( GA_ATTRIB_POINT, attribName, 1 );		// tuple of size 1
		if ( attRef1.isInvalid() || attRef2.isInvalid() )
        {
			addError( SOP_ATTRIBUTE_INVALID, "You have provided an attribute that does not exist in one or both of your input geometries." );
		}  // if
		
		// Create the new point group for matching points in the first input geometry (gdp)
		GA_PointGroup* matchingPointsGrp = gdp->newPointGroup( matchingPointsGroupName );
		
		// Create a list of the attribName values of all the points in input geometry two
        /*
		numPoints2 = gdp2->points()->entries();
		int* attrValues2 = new int[numPoints2];
		int i = 0;
		FOR_ALL_GPOINTS( gdp2, pt2 )
		{
			attrValues2[i] = pt2->getValue<int>( attRef2 );
			i++;
		}  // FOR_ALL_GPOINTS
		*/
		
        // Parse through each point in the first input geometry and add it to the matching points group
		//   if it has a matching attribute value for attribName in input geometry two.
        GEO_Point* pt1;
        GA_FOR_ALL_OPT_GROUP_POINTS( gdp, group, pt1 )
        {
			int objid1 = pt1->getValue<int>( attRef1 );
			
			const GEO_Point* pt2;
			GA_FOR_ALL_GPOINTS( gdp2, pt2 )
			{
				int objid2 = pt2->getValue<int>( attRef2 );
				if ( objid1 == objid2 )
				{
					matchingPointsGrp->add( pt1 );
					break;
				}  // if
			}  // FOR_ALL_GPOINTS
            
        }  // FOR_ALL_GPOINTS
        
    }  // if
    
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    //gdp->notifyCache(GU_CACHE_ALL);
    
    return error();
}

