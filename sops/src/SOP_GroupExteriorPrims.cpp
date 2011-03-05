#include "SOP_GroupExteriorPrims.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
#include <DOP/DOP_Engine.h>
#include <DOP/DOP_Parent.h>
#include <GB/GB_EdgeGroup.h>
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
    PRM_Name("intprimgroup", "Interior Prims Group"),
    PRM_Name("extprimgroup", "Exterior Prims Group"),
};

static PRM_Default          defEighteen(18);
static PRM_Default          defEmptyString( 0, "" );

PRM_Template
SOP_GroupExteriorPrims::myTemplateList[] = {
    PRM_Template( PRM_STRING, 1, &names[0], &defEmptyString ),
    PRM_Template( PRM_STRING, 1, &names[1], &defEmptyString ),
    PRM_Template(),
};




void
newSopOperator( OP_OperatorTable *table )
{
     table->addOperator(
                    new OP_Operator("groupexteriorprims",
					"Group Exterior Prims",
					SOP_GroupExteriorPrims::myConstructor,
					SOP_GroupExteriorPrims::myTemplateList,
					1,
					1,
					0)
                        );
}




OP_Node *
SOP_GroupExteriorPrims::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_GroupExteriorPrims(net, name, op);
}




SOP_GroupExteriorPrims::SOP_GroupExteriorPrims( OP_Network *net, const char *name, OP_Operator *op )
	: SOP_Node(net, name, op), myGroup(0)
{
}

SOP_GroupExteriorPrims::~SOP_GroupExteriorPrims() {}




OP_ERROR SOP_GroupExteriorPrims::cookMySop( OP_Context &context )
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
        UT_String intPrimGrp = INTPRIMGROUP(t);
        const GB_PrimitiveGroup* interiorPrimGroup = parsePrimitiveGroups( (const char*)intPrimGrp, gdp );
        
        // Get the name of the exterior primitive group
        UT_String extPrimGrp = EXTPRIMGROUP(t);
        const GB_PrimitiveGroup* exteriorPrimGroup = parsePrimitiveGroups( (const char*)extPrimGrp, gdp );
        
        // Create the empty edge list
        GB_EdgeGroup edgeList( (*gdp), "interiorEdges" );
        
        // Create the empty prim group of prim to be deleted
        GB_PrimitiveGroup* deletePrimsGrp = gdp->newPrimitiveGroup( extPrimGrp );
        
        // Parse through each primitive and attach its edges to the edge list
        GEO_Primitive* prim;
        FOR_ALL_GROUP_PRIMITIVES( gdp, interiorPrimGroup, prim )
        {
            if ( prim->getPrimitiveId() & GEOPRIMPOLY )
            {
                // THIS ASSUMES A TRIANGULAR PRIM
                int numVertices = prim->getVertexCount();
                for ( int i = 0; i < numVertices; i++ )
                {
                    GEO_Point* p1 = prim->getVertex(i).getPt();
                    GEO_Point* p2 = prim->getVertex( (i+1)%numVertices ).getPt();
                    
                    //if ( prim->hasEdge( *p1, *p2 ) )
                    //{
                    GB_Edge curEdge( p1, p2 );
                    if ( !edgeList.contains(curEdge) )
                    {
                        edgeList.add( p1, p2 );
                    }  // if
                    //}  // if
                }  // for v
            }  // if
        }  // FOR_ALL_GROUP_PRIMITIVES
        
        // For all primitives in the geometry, check which ones are not surrounded by the grouped edges
        GB_EdgeGroup tmp( *gdp, "tmp" );
        FOR_ALL_PRIMITIVES( gdp, prim )
        {
            bool doDeletion = false;
            
            int numVertices = prim->getVertexCount();
            for ( int i = 0; i < numVertices; i++ )
            {
                GEO_Point* p1 = prim->getVertex(i).getPt();
                GEO_Point* p2 = prim->getVertex( (i+1)%numVertices ).getPt();
                
                GB_Edge curEdge( p1, p2 );
                
                if ( !tmp.contains(curEdge) )
                    tmp.add( p1, p2 );
                
                // If the edge group does not contain every edge on the prim, then the prim needs to be deleted
                if ( !edgeList.contains( curEdge ) )
                {
                    doDeletion = true;
                }  // if
            }  // for i
            
            if ( doDeletion )
            {
                deletePrimsGrp->add( prim );
            }  // if
        }  // FOR_ALL_PRIMITIVES
    }  // if
    
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    //gdp->notifyCache(GU_CACHE_ALL);
    
    return error();
}
