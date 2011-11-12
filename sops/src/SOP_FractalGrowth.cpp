#include "SOP_FractalGrowth.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
//#include <UT/UT_Matrix3.h>
//#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <GU/GU_PrimSphere.h>
#include <PRM/PRM_Default.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <SOP/SOP_Guide.h>
#include <UT/UT_Vector4.h>
#include <UT/UT_Vector4Array.h>

#include <stdlib.h>
#include <iostream>
using namespace std;




static PRM_Name        names[] = {
    PRM_Name("rad",	"Radius"),
    PRM_Name("numpoints", "Num Points"),
    //PRM_Name("phase",	"Phase"),
    //PRM_Name("period",	"Period"),
};

static PRM_Default         defEighteen(18);
static PRM_Range           defNumParticlesRange( PRM_RANGE_UI, 0, PRM_RANGE_UI, 50 );

PRM_Template
SOP_FractalGrowth::myTemplateList[] = {
    PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0, &PRMscaleRange),
    PRM_Template(PRM_INT_J,	1, &names[1], &defEighteen, 0, &defNumParticlesRange),
    PRM_Template(),
};




void
newSopOperator( OP_OperatorTable *table )
{
     table->addOperator(new OP_Operator("fractalgrowth",
					"Fractal Growth",
					 SOP_FractalGrowth::myConstructor,
					 SOP_FractalGrowth::myTemplateList,
					 1,
					 1,
					 0));
}




OP_Node *
SOP_FractalGrowth::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_FractalGrowth(net, name, op);
}




SOP_FractalGrowth::SOP_FractalGrowth( OP_Network *net, const char *name, OP_Operator *op )
	: SOP_Node(net, name, op), myGroup(0)
{
}

SOP_FractalGrowth::~SOP_FractalGrowth() {}




UT_Vector4 SOP_FractalGrowth::computeChildPosition( UT_Vector4 p1, UT_Vector4 p2, UT_Vector4 norm, fpreal radius )
{
    UT_Vector4 p2p1 = p1 - p2;
    fpreal p2p1Mag = p2p1.length();
    fpreal hypotenuse = 2 * radius;
    fpreal adjacent = p2p1Mag / 2.0;
    
    UT_Vector4 p2p1Norm;
    if ( adjacent >= hypotenuse )
    {
        p2p1Norm = ( p1 + p2 ) / 2.0;
    }
    else
    {
        p2p1.normalize();
        p2p1Norm = p2p1;
        fpreal costheta = adjacent / hypotenuse;
        fpreal theta = acos( costheta );
        fpreal sintheta = sin( theta );
        fpreal negcostheta = -1 * costheta;
        fpreal negsintheta = -1 * sintheta;
        
        norm.normalize();
        
        //UT_Matrix4 rotX( 1,    0,        0,           0,
        //                 0,    costheta, negsintheta, 0,
        //                 0,    sintheta, costheta,    0,
        //                 0,    0,        0,           1  );
        
        UT_Matrix4 rotY;
        if ( cross(p2p1,norm).y() < 0 )
        {
            rotY = UT_Matrix4( costheta,     0,    sintheta,    0,
                               0,            1,    0,           0,
                               negsintheta,  0,    costheta,    0,
                               0,            0,    0,           1  );
            
            p2p1Norm.rowVecMult( rotY );
            p2p1Norm *= hypotenuse;
            p2p1Norm = p2 + p2p1Norm;
            
        }  // if
        else
        {
            rotY = UT_Matrix4( negcostheta,  0,    negsintheta, 0,
                               0,            1,    0,           0,
                               sintheta,     0,    negcostheta, 0,
                               0,            0,    0,           1  );
            
            p2p1Norm.rowVecMult( rotY );
            p2p1Norm *= hypotenuse;
            p2p1Norm = p1 + p2p1Norm;
            
        }  // else
        
        //UT_Matrix4 rotZ( costheta, negsintheta, 0, 0,
        //                 sintheta, costheta,    0, 0,
        //                 0,        0,           1, 0,
        //                 0,        0,           0, 1  );
        
    }
        
    return p2p1Norm;

}  // computeChildPosition()




bool SOP_FractalGrowth::intersectRaySphere( UT_Vector4 rayOrigin, UT_Vector4 ray, UT_Vector4 sphCenter, fpreal radius )
{
    ray.normalize();
    
    UT_Vector4 sphVector = sphCenter - rayOrigin;
    
    fpreal distFromSphCenterToRay = ( ray.dot(sphVector) * ray - sphVector ).length();
    
    if ( distFromSphCenterToRay < radius )
        return true;
    else
        return false;

}  // intersectRaySphere




OP_ERROR SOP_FractalGrowth::cookMySop( OP_Context &context )
{
    GEO_Point   *ppt;
    
    int sphRad = RADIUS();
    int numSpheresToPopulate = NUMPOINTS();
    
    // Before we do anything, we must lock our inputs.  Before returning,
    //	we have to make sure that the inputs get unlocked.
    if ( lockInputs(context) >= UT_ERROR_ABORT )
        return error();
    
    float t = context.myTime;
	
	srand ( 5 );
    
    // Duplicate our incoming geometry with the hint that we only
    // altered points.  Thus if we our input was unchanged we can
    // easily roll back our changes by copying point values.
    duplicatePointSource(0, context);
    
    if ( error() < UT_ERROR_ABORT )
    {
		for ( int i = 0; i < numSpheresToPopulate; i++ )
        {
			// Get the number of prims
			GEO_PrimList& prims = gdp->primitives();
			int numPrims = prims.entries();
			
			// Pick a random prim
			int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
			int randPrimIndex = randomNumber % numPrims;
			GEO_Primitive* prim = prims(randPrimIndex);
			
			// Get its two vertices
			GEO_Vertex vert0 = prim->getVertexElement(0);
			GEO_Vertex vert1 = prim->getVertexElement(1);
			
			// Get its normal
			GEO_AttributeHandle normAttrib = gdp->getPrimAttribute( "edgeNormal" );
			normAttrib.setElement( prim );
			UT_Vector4 edgeNormal = normAttrib.getV4();
			
			// Get the prim's points
			GEO_Point* pt0 = vert0.getPt();
			GEO_Point* pt1 = vert1.getPt();
			UT_Vector3 pt0Pos = pt0->getPos3();
			UT_Vector3 pt1Pos = pt1->getPos3();
			
			// Get the intermediate point of the current prim, if it exists, otherwise just use pt1
			float x1, y1, z1;
			GA_RWAttributeRef numintermediatepts_index = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "numIntermediatePts", 1 );
			int numIntermediatePts = prim->getValue<int>( numintermediatepts_index );		// Gets the number of intermediate points on the chosen prim
			if ( numIntermediatePts > 0 )
			{
				GA_RWAttributeRef intermediateptpos_index1 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt1", 3 );	// Get the first intermediate point
				x1 = prim->getValue<float>( intermediateptpos_index1, 0 );
				y1 = prim->getValue<float>( intermediateptpos_index1, 1 );
				z1 = prim->getValue<float>( intermediateptpos_index1, 2 );
			}  // if
			else
			{
				x1 = pt1->getPos()[0];
				y1 = pt1->getPos()[1];
				z1 = pt1->getPos()[2];
			}  // else
			
			// Create the new sphere's point
			//UT_Vector4 childPos = computeChildPosition( pt0->getPos(), pt1->getPos(), edgeNormal, 1 );
			//UT_Vector4 childPos = computeChildPosition( UT_Vector4(x0,y0,z0,1), UT_Vector4(x1,y1,z1,1), edgeNormal, 1 );
			UT_Vector4 newPtPos = computeChildPosition( pt0->getPos(), UT_Vector4(x1,y1,z1,1), edgeNormal, 1 );
			GEO_Point* newPt = gdp->appendPointElement();
			newPt->setPos( newPtPos );
			
			
			// Create a prim between each of the prim's old points and the newly created point (keep the edge directionality)
			//   Note: The number of intermediate points attribute (numIntermediatePts) defaults to 0, so we don't have to explicitly set it
			GU_PrimPoly* newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
			newPrim0->appendVertex(pt0);
            newPrim0->appendVertex(newPt);
			
			GU_PrimPoly* newPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
			newPrim1->appendVertex(newPt);
            newPrim1->appendVertex(pt1);
			
			
			// If there were two intermediate points in the old edge (prim), add the second intermediate point to newPrim1
			if ( numIntermediatePts == 2 )
			{
				GA_RWAttributeRef intermediateptnum_index1 = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "intermediatePtNum1", 1 );
				GA_RWAttributeRef intermediateptpos_index1 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt1", 3 );
				GA_RWAttributeRef intermediateptnum_index2 = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "intermediatePtNum2", 1 );
				GA_RWAttributeRef intermediateptpos_index2 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt2", 3 );
				
				// Get prim's second intermediate point info
				int intPtNum = prim->getValue<int>( intermediateptnum_index2 );
				UT_Vector3 intPtPos = prim->getValue<UT_Vector3>( intermediateptpos_index2 );
				
				// Set newPrim1's first intermediate point info
				newPrim1->setValue<int>( intermediateptnum_index1, intPtNum );
				newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, intPtPos );
				newPrim1->setValue<int>( numintermediatepts_index, 1 );
			}  // if
			
			/*
			// If the length of the new prim is long enough to fit a sphere between, add intermediate points to prim1
			UT_Vector3 prim1Vector = pt1->getPos() - newPt->getPos();
			float prim1Length = prim1Vector.length();
			//if ( dotBetweenEdges < -0.5 )		// If the angle is greater than 120 degrees
			if ( prim1Length > 3.4641 * sphRad )
			{
				for ( int i = 0; i < numIntermediatePts-1; i++ )
				{
					char tmp[181]; sprintf( tmp, "%s%d", "intermediatePtPos", i );
					UT_String intermediatePtPosAttrName = tmp;
					char tmp1[181]; sprintf( tmp1, "%s%d", "intermediatePtPos", i+1 );
					UT_String intermediatePtPosAttrName1 = tmp1;
					
					GA_RWAttributeRef intermediateptpos_index = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, intermediatePtPosAttrName, 3 );
					GA_RWAttributeRef intermediateptpos_index1 = gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, intermediatePtPosAttrName1, 3, GA_Defaults(0.0) );
					
					// Get the prim's current intermediatePtPos, the (i+1)th intermediate point on the original prim
					UT_Vector3 curIntermediatePtPos = prim->getValue<UT_Vector3>( intermediateptpos_index1 );
					
					// Set newPrim1's ith intermediatePtPos
					newPrim1->setValue<UT_Vector3>( intermediateptpos_index, curIntermediatePtPos );
				}  // for i
			
				if ( numIntermediatePts > 0 )
				{
					newPrim1->setValue<float>( numintermediatepts_index, numIntermediatePts-1 );
				}  // else if
			}  // if
			*/
			
			
			// Get the prim's neighboring prims
			//    THERE'S A SIMPLER WAY TO DO THIS IF WE KNOW THE ORDERING THAT GETPRIMITIVESREFERENCINGPOINT IS RETURNING THE PRIMS
			GA_OffsetArray connectedPrims;
			GEO_Primitive* primNeighbor0;
			GEO_Primitive* primNeighbor1;
			
			gdp->getPrimitivesReferencingPoint( connectedPrims, gdp->pointOffset(pt0->getMapIndex()) );
			
			if ( gdp->primitiveIndex(connectedPrims[0]) == prim->getMapIndex() )
			{
				primNeighbor0 = gdp->primitives()[gdp->primitiveIndex(connectedPrims[1])];
			}  // if
			else
			{
				primNeighbor0 = gdp->primitives()[gdp->primitiveIndex(connectedPrims[0])];
			}  // else
			
			gdp->getPrimitivesReferencingPoint( connectedPrims, gdp->pointOffset(pt1->getMapIndex()) );
			
			if ( gdp->primitiveIndex(connectedPrims[0]) == prim->getMapIndex() )
				primNeighbor1 = gdp->primitives()[gdp->primitiveIndex(connectedPrims[1])];
			else
				primNeighbor1 = gdp->primitives()[gdp->primitiveIndex(connectedPrims[0])];
			
			
			
			
			// Compute the new prims' normals and set their "edgeNormal" attribute
			GA_RWAttributeRef norm_index = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "edgeNormal", 3 );
			
			UT_Vector3 edgeVector0 = newPt->getPos3() - pt0->getPos3();
			edgeVector0.normalize();
			UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
			newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
			newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
			newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
			
			UT_Vector3 edgeVector1 = pt1->getPos3() - newPt->getPos3();
			edgeVector1.normalize();
			UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
			newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
			newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
			newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
			
			// Delete the prim
			gdp->deletePrimitive( *prim );
			
			
			GA_RWAttributeRef intermediateptpos_index1 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt1", 3 );
			GA_RWAttributeRef intermediateptpos_index2 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt2", 3 );
			GA_RWAttributeRef intermediateptnum_index1 = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "intermediatePtNum1", 1 );
			GA_RWAttributeRef intermediateptnum_index2 = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "intermediatePtNum2", 1 );
			
			
			// PRIM 0
			// Compute whether or not newprim0 and primNeighbor0 are forming a concavity
			//    (If prim0's neighbor's unshared vertex is within the halfspace of prim0,
			//    meaning the dot product of the normal with the vector from the prim0's vertex to the neighbor0's unshared vertex is greater than zero,
			//    then prim0 and primNeighbor0 are creating a concavity)
			GEO_Vertex neighborVert0 = primNeighbor0->getVertexElement(0);		// Neighbor's unshared vertex is the neighbor prim's first vertex
			GEO_Point* neighborPt0 = neighborVert0.getPt();
			UT_Vector3 neighborPos0 = neighborPt0->getPos3();
			//UT_Vector3 newPtPos = newPt->getPos3();
			float dotProd0 = normal0.dot( neighborPos0 - newPtPos );
			
			// If prim0 has a concavity with its neighbor:
			//    Create a prim between prim0's pt1 and its neighbor's unshared point
			//    If the triangle formed from the filled concavity has an angle greater than 120 degrees
			//       Keep the whole triangle around (all 3 prims)
			if ( dotProd0 > 0 )
			{
				// If there is less than two intermediate point in the neighboring prim
				//   newPrim0 = join( pNeighbor0, newP )
				//   If there are no intermediate points in primNeighbor0
				//     If pt0 and pNeighbor0 are far enough apart to insert an intermediate point		IF-STATEMENT MAY BE UNNECESSARY, SINCE THIS MAY ALWAYS BE TRUE (PROVE?)
				//       newPrim0.ptIntermediate = pt0
				//       newPrim0.numIntermediatePts = 1
				//     Else
				//       newPrim0.numIntermediatePts = 0
				//   Elif there is one intermediate point in primNeighbor0
				//     newPrim0.ptIntermediate = primNeighbor0.ptIntermediate
				//     newPrim0.numIntermediatePts = 2
				//   Delete primNeighbor0
				// Else
				//   See pseudocode below
				int numNeighborIntermediatePts = primNeighbor0->getValue<int>( numintermediatepts_index );
				if ( numNeighborIntermediatePts < 2 )
				{
					gdp->deletePrimitive( *newPrim0 );
					
					newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
					newPrim0->appendVertex(neighborPt0);
					newPrim0->appendVertex(newPt);
					
					//GA_RWAttributeRef intermediateptpos_index1 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt1", 3 );
					if ( numNeighborIntermediatePts == 0 )
					{
						UT_Vector3 newEdge = newPtPos - neighborPos0;
						float newEdgeLength = newEdge.length();
						if ( newEdgeLength > 3.4641 * sphRad )
						{
							newPrim0->setValue<int>( intermediateptnum_index1, pt0->getMapIndex() );
							newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, pt0Pos );
							newPrim0->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						else
						{
							newPrim0->setValue<int>( numintermediatepts_index, 0 );
						}  // else
					}  // if
					else		// numNeighborIntermediatePts == 1
					{
						UT_Vector3 neighIntermediatePtPos = primNeighbor0->getValue<UT_Vector3>( intermediateptpos_index1 );
						int neighIntermediatePtNum = primNeighbor0->getValue<int>( intermediateptnum_index1 );
						
						newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, neighIntermediatePtPos );
						newPrim0->setValue<UT_Vector3>( intermediateptpos_index2, pt0Pos );
						newPrim0->setValue<int>( intermediateptnum_index1, neighIntermediatePtNum );
						newPrim0->setValue<int>( intermediateptnum_index2, pt0->getMapIndex() );
						
						newPrim0->setValue<int>( numintermediatepts_index, 2 );
					}  // else
					
					UT_Vector3 edgeVector0 = newPt->getPos3() - neighborPt0->getPos3();
					edgeVector0.normalize();
					UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
					newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
					newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
					newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
					
					gdp->deletePrimitive( *primNeighbor0 );
				}  // if
				else			// There are 2 intermediate granules in primNeighbor0
				{
					// If the angle between the newPrim0 and the neighborPrim0 is LESS than 120 degrees
					//   splitPrim0 = Join( neighborPt0, intermediatePt2 )
					//   splitPrim1 = Join( intermediatePt2, newPt0 )
					//   If neighborPt0 and intermediatePt2 are far enough apart
					//     splitPrim0.intermediatePt1 = intermediatePt1
					//   If newPt and intermediatePt are far enough apart
					//     splitPrim1.intermediatePt = pt0
					//   Delete newPrim0 and neighborPrim0
					UT_Vector3 edge1 = neighborPos0 - pt0Pos;
					UT_Vector3 edge2 = newPtPos - pt0Pos;
					edge1.normalize();
					edge2.normalize();
					float dotBetweenEdges = edge1.dot( edge2 );
					if ( dotBetweenEdges > -0.5 )		// If the angle is less than 120 degrees
					{
						int intPt1Num = primNeighbor0->getValue<int>( intermediateptnum_index1 );
						int intPt2Num = primNeighbor0->getValue<int>( intermediateptnum_index2 );
						GEO_Point* intermediatePt1 = gdp->points()[intPt1Num];
						GEO_Point* intermediatePt2 = gdp->points()[intPt2Num];
						UT_Vector3 intPt2Pos = intermediatePt2->getPos3();
						
						GU_PrimPoly* splitPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						splitPrim0->appendVertex(neighborPt0);
						splitPrim0->appendVertex(intermediatePt2);
						
						GU_PrimPoly* splitPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						splitPrim1->appendVertex(intermediatePt2);
						splitPrim1->appendVertex(newPt);
						
						UT_Vector3 vecNeighToInt = intPt2Pos - neighborPos0;
						if ( vecNeighToInt.length() > 3.4641 * sphRad )
						{
							splitPrim0->setValue<int>( intermediateptnum_index1, intermediatePt1->getMapIndex() );
							splitPrim0->setValue<UT_Vector3>( intermediateptpos_index1, intermediatePt1->getPos3() );
							splitPrim0->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						
						UT_Vector3 vecIntToNewPt = intPt2Pos - newPtPos;
						if ( vecIntToNewPt.length() > 3.4641 * sphRad )
						{
							splitPrim1->setValue<int>( intermediateptnum_index1, pt0->getMapIndex() );
							splitPrim1->setValue<UT_Vector3>( intermediateptpos_index1, pt0Pos );
							splitPrim1->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						
						gdp->deletePrimitive( *newPrim0 );
						gdp->deletePrimitive( *primNeighbor0 );
					}  // if
				}  // else
				
				
				
				
				
				/*
				GU_PrimPoly* convexPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
				convexPrim0->appendVertex(neighborPt0);
				convexPrim0->appendVertex(newPt);
				
				// Get the angle between prim0 and prim0's neighbor
				UT_Vector3 edge1 = newPtPos - pt0Pos;
				edge1.normalize();
				UT_Vector3 edge2 = neighborPos0 - pt0Pos;
				edge2.normalize();
				float dotBetweenEdges = edge1.dot( edge2 );
				UT_Vector3 newEdge = newPtPos - neighborPos0;
				float newEdgeLength = newEdge.length();
				//if ( dotBetweenEdges < -0.5 )		// If the angle is greater than 120 degrees
				if ( newEdgeLength > 3.4641 * sphRad )
				{
					// Get the number of intermediate points
					GA_RWAttributeRef numintermediatepts_index = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "numIntermediatePts", 1 );
					int numIntermediatePts = primNeighbor0->getValue<float>( numintermediatepts_index, 0 );
					
					// Copy over the neighboring prim's intermediate points
					int i = 0;
					while ( i < numIntermediatePts )
					{
						// Get the current intermediatePtPos's attribute name
						char tmp[181]; sprintf( tmp, "%s%d", "intermediatePtPos", i );
						UT_String intermediatePtPosAttrName = tmp;
						
						// Create the intermediate point position attribute.
						//   If it already exists, this will return the existing attribute (this is the default behavior)
						//GA_RWAttributeRef intermediateptpos_index = gdp->addFloatTuple( GA_ATTRIB_POINT, intermediatePtPosAttrName, 3, GA_Defaults(0.0) );
						GA_RWAttributeRef intermediateptpos_index = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, intermediatePtPosAttrName, 3 );
						
						// Get the primNeighbor0's current intermediatePtPos
						UT_Vector3 curIntermediatePtPos = primNeighbor0->getValue<UT_Vector3>( intermediateptpos_index );
						
						// Set the convex prim's current intermediatePtPos
						convexPrim0->setValue<UT_Vector3>( intermediateptpos_index, curIntermediatePtPos );
						
						i += 1;
					}  // while
					
					// Add pt0's position to the list of convex prim's intermediate positions
					char tmp[181]; sprintf( tmp, "%s%d", "intermediatePtPos", i );
					UT_String intermediatePtPosAttrName = tmp;
					
					GA_RWAttributeRef intermediateptpos_index = gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, intermediatePtPosAttrName, 3, GA_Defaults(0.0) );
					
					convexPrim0->setValue<UT_Vector3>( intermediateptpos_index, pt0Pos );
					
					convexPrim0->setValue<float>( numintermediatepts_index, numIntermediatePts+1 );
					
					// Give the prim its seed creation points for birthing a new particle
					//convexPrim0->setValue<float>( pt0_index, pt0Pos[0], 0 );
					//convexPrim0->setValue<float>( pt0_index, pt0Pos[1], 1 );
					//convexPrim0->setValue<float>( pt0_index, pt0Pos[2], 2 );
					//convexPrim0->setValue<float>( pt1_index, newPtPos[0], 0 );
					//convexPrim0->setValue<float>( pt1_index, newPtPos[1], 1 );
					//convexPrim0->setValue<float>( pt1_index, newPtPos[2], 2 );
				}  // if
				else
				{
					convexPrim0->setValue<float>( numintermediatepts_index, 0 );		// Clear out the intermediate points
					// Give the prim its seed creation points for birthing a new particle
					//convexPrim0->setValue<float>( pt0_index, neighborPos0[0], 0 );
					//convexPrim0->setValue<float>( pt0_index, neighborPos0[1], 1 );
					//convexPrim0->setValue<float>( pt0_index, neighborPos0[2], 2 );
					//convexPrim0->setValue<float>( pt1_index, newPtPos[0], 0 );
					//convexPrim0->setValue<float>( pt1_index, newPtPos[1], 1 );
					//convexPrim0->setValue<float>( pt1_index, newPtPos[2], 2 );
				}  // else
				*/
				
				
				// Delete the new prims to be replaced by the convex prim
				//gdp->deletePrimitive( *newPrim0 );
				//gdp->deletePrimitive( *primNeighbor0 );
			}  // if
			
			
			
			// PRIM 1
			// Compute whether or not newprim1 and primNeighbor1 are forming a concavity
			//    (If prim1's neighbor's unshared vertex is within the halfspace of prim1
			//    meaning the dot product of the normal with the vector from the prim1's vertex to the neighbor1's unshared vertex is greater than zero,
			//    then prim1 and primNeighbor1 are creating a concavity)
			GEO_Vertex neighborVert1 = primNeighbor1->getVertexElement(1);		// Neighbor's unshared vertex is the neighbor prim's second vertex
			GEO_Point* neighborPt1 = neighborVert1.getPt();
			UT_Vector3 neighborPos1 = neighborPt1->getPos3();
			float dotProd1 = normal1.dot( neighborPos1 - newPtPos );
			
			// If prim1 has a concavity with its neighbor:
			//    Create a prim between prim1's pt1 and its neighbor's unshared point
			//    If the triangle formed from the filled concavity has an angle greater than 120 degrees
			//       Keep the whole triangle around (all 3 prims)
			if ( dotProd1 > 1 )
			{
				// If there is less than two intermediate point in the neighboring prim
				//   newPrim0 = join( pNeighbor0, newP )
				//   If there are no intermediate points in primNeighbor0
				//     If pt0 and pNeighbor0 are far enough apart to insert an intermediate point		IF-STATEMENT MAY BE UNNECESSARY, SINCE THIS MAY ALWAYS BE TRUE (PROVE?)
				//       newPrim0.ptIntermediate = pt0
				//       newPrim0.numIntermediatePts = 1
				//     Else
				//       newPrim0.numIntermediatePts = 0
				//   Elif there is one intermediate point in primNeighbor0
				//     newPrim0.ptIntermediate = primNeighbor0.ptIntermediate
				//     newPrim0.numIntermediatePts = 2
				//   Delete primNeighbor0
				// Else
				//   newPrim0 = join( pt0, newP )
				int numNeighborIntermediatePts = primNeighbor1->getValue<int>( numintermediatepts_index );
				if ( numNeighborIntermediatePts < 2 )
				{
					gdp->deletePrimitive( *newPrim1 );
					
					newPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
					newPrim1->appendVertex(newPt);
					newPrim1->appendVertex(neighborPt1);
					
					//GA_RWAttributeRef intermediateptpos_index1 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt1", 3 );
					if ( numNeighborIntermediatePts == 0 )
					{
						UT_Vector3 newEdge = newPtPos - neighborPos1;
						float newEdgeLength = newEdge.length();
						if ( newEdgeLength > 3.4641 * sphRad )
						{
							newPrim1->setValue<int>( intermediateptnum_index1, pt1->getMapIndex() );
							newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, pt1Pos );
							newPrim1->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						else
						{
							newPrim1->setValue<int>( numintermediatepts_index, 0 );
						}  // else
					}  // if
					else		// numNeighborIntermediatePts == 1
					{
						UT_Vector3 neighIntermediatePtPos = primNeighbor1->getValue<UT_Vector3>( intermediateptpos_index1 );
						float neighIntermediatePtNum = primNeighbor1->getValue<int>( intermediateptnum_index1 );
						
						newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, pt1Pos );
						newPrim1->setValue<UT_Vector3>( intermediateptpos_index2, neighIntermediatePtPos );
						newPrim1->setValue<int>( intermediateptnum_index1, pt1->getMapIndex() );
						newPrim1->setValue<int>( intermediateptnum_index2, neighIntermediatePtNum );
						
						newPrim1->setValue<int>( numintermediatepts_index, 2 );
					}  // else
					
					UT_Vector3 edgeVector1 = neighborPt1->getPos3() - newPt->getPos3();
					edgeVector1.normalize();
					UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
					newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
					newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
					newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
					
					gdp->deletePrimitive( *primNeighbor1 );
				}  // if
				else
				{
					// If the angle between the newPrim0 and the neighborPrim0 is LESS than 120 degrees
					//   splitPrim0 = Join( neighborPt0, intermediatePt2 )
					//   splitPrim1 = Join( intermediatePt2, newPt0 )
					//   If neighborPt0 and intermediatePt2 are far enough apart
					//     splitPrim0.intermediatePt1 = intermediatePt1
					//   If newPt and intermediatePt are far enough apart
					//     splitPrim1.intermediatePt = pt0
					//   Delete newPrim0 and neighborPrim0
					UT_Vector3 edge1 = neighborPos1 - pt1Pos;
					UT_Vector3 edge2 = newPtPos - pt1Pos;
					edge1.normalize();
					edge2.normalize();
					float dotBetweenEdges = edge1.dot( edge2 );
					if ( dotBetweenEdges > -0.5 )		// If the angle is less than 120 degrees
					{
						int intPt1Num = primNeighbor1->getValue<int>( intermediateptnum_index1 );
						int intPt2Num = primNeighbor1->getValue<int>( intermediateptnum_index2 );
						GEO_Point* intermediatePt1 = gdp->points()[intPt1Num];
						GEO_Point* intermediatePt2 = gdp->points()[intPt2Num];
						UT_Vector3 intPt1Pos = intermediatePt1->getPos3();
						
						GU_PrimPoly* splitPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						splitPrim0->appendVertex(newPt);
						splitPrim0->appendVertex(intermediatePt1);
						
						GU_PrimPoly* splitPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						splitPrim1->appendVertex(intermediatePt1);
						splitPrim1->appendVertex(neighborPt1);
						
						UT_Vector3 vecIntToNewPt = intPt1Pos - newPtPos;
						if ( vecIntToNewPt.length() > 3.4641 * sphRad )
						{
							splitPrim0->setValue<int>( intermediateptnum_index1, pt1->getMapIndex() );
							splitPrim0->setValue<UT_Vector3>( intermediateptpos_index1, pt1Pos );
							splitPrim0->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						
						UT_Vector3 vecNeighToInt = intPt1Pos - neighborPos1;
						if ( vecNeighToInt.length() > 3.4641 * sphRad )
						{
							splitPrim1->setValue<int>( intermediateptnum_index1, intermediatePt2->getMapIndex() );
							splitPrim1->setValue<UT_Vector3>( intermediateptpos_index1, intermediatePt2->getPos3() );
							splitPrim1->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						
						gdp->deletePrimitive( *newPrim1 );
						gdp->deletePrimitive( *primNeighbor1 );
					}  // if
				}  // else
				
				
				/*
				GU_PrimPoly* convexPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
				convexPrim1->appendVertex(newPt);
				convexPrim1->appendVertex(neighborPt1);
				
				// Get the angle between prim1 and prim1's neighbor
				UT_Vector3 edge1 = newPtPos - pt1Pos;
				edge1.normalize();
				UT_Vector3 edge2 = neighborPos1 - pt1Pos;
				edge2.normalize();
				float dotBetweenEdges = edge1.dot( edge2 );
				UT_Vector3 newEdge = newPtPos - neighborPos0;
				float newEdgeLength = newEdge.length();
				//if ( dotBetweenEdges < -0.5 )		// If the angle is greater than 120 degrees
				if ( newEdgeLength > 3.4641 * sphRad )
				{
					// Get the number of intermediate points
					GA_RWAttributeRef numintermediatepts_index = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "numIntermediatePts", 1 );
					int numIntermediatePts = primNeighbor1->getValue<float>( numintermediatepts_index, 0 );
					
					// Add pt0's position to the list of convex prim's intermediate positions
					char tmp[181]; sprintf( tmp, "%s%d", "intermediatePtPos", 0 );
					UT_String intermediatePtPosAttrName = tmp;
					GA_RWAttributeRef intermediateptpos_index = gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, intermediatePtPosAttrName, 3, GA_Defaults(0.0) );
					
					convexPrim1->setValue<UT_Vector3>( intermediateptpos_index, pt1Pos );
					
					// Copy over the neighboring prim's intermediate points
					int i = 0;
					while ( i < numIntermediatePts )
					{
						// Get the current intermediatePtPos's attribute name
						char tmp[181]; sprintf( tmp, "%s%d", "intermediatePtPos", i );
						UT_String intermediatePtPosAttrName = tmp;
						char tmp1[181]; sprintf( tmp1, "%s%d", "intermediatePtPos", i+1 );
						UT_String intermediatePtPosAttrName1 = tmp1;
						
						// Create the intermediate point position attribute.
						//   If it already exists, this will return the existing attribute (this is the default behavior)
						//GA_RWAttributeRef intermediateptpos_index = gdp->addFloatTuple( GA_ATTRIB_POINT, intermediatePtPosAttrName, 3, GA_Defaults(0.0) );
						GA_RWAttributeRef intermediateptpos_index = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, intermediatePtPosAttrName, 3 );
						GA_RWAttributeRef intermediateptpos_index1 = gdp->addFloatTuple( GA_ATTRIB_PRIMITIVE, intermediatePtPosAttrName1, 3, GA_Defaults(0.0) );
						
						// Get the primNeighbor0's current intermediatePtPos
						UT_Vector3 curIntermediatePtPos = primNeighbor1->getValue<UT_Vector3>( intermediateptpos_index );
						
						// Set the convex prim's current intermediatePtPos
						convexPrim1->setValue<UT_Vector3>( intermediateptpos_index1, curIntermediatePtPos );
						
						i += 1;
					}  // while
					
					convexPrim1->setValue<float>( numintermediatepts_index, numIntermediatePts+1 );
					
					// Give the prim its seed creation points for birthing a new particle
					//convexPrim1->setValue<float>( pt0_index, newPtPos[0], 0 );
					//convexPrim1->setValue<float>( pt0_index, newPtPos[1], 1 );
					//convexPrim1->setValue<float>( pt0_index, newPtPos[2], 2 );
					//convexPrim1->setValue<float>( pt1_index, pt1Pos[0], 0 );
					//convexPrim1->setValue<float>( pt1_index, pt1Pos[1], 1 );
					//convexPrim1->setValue<float>( pt1_index, pt1Pos[2], 2 );
				}  // if
				else
				{
					convexPrim1->setValue<float>( numintermediatepts_index, 0 );		// Clear out the intermediate points
					// Give the prim its seed creation points for birthing a new particle
					//convexPrim1->setValue<float>( pt0_index, newPtPos[0], 0 );
					//convexPrim1->setValue<float>( pt0_index, newPtPos[1], 1 );
					//convexPrim1->setValue<float>( pt0_index, newPtPos[2], 2 );
					//convexPrim1->setValue<float>( pt1_index, neighborPos1[0], 0 );
					//convexPrim1->setValue<float>( pt1_index, neighborPos1[1], 1 );
					//convexPrim1->setValue<float>( pt1_index, neighborPos1[2], 2 );
				}  // else
				*/
				
				// Delete the new prims to be replaced by the convex prim
				//gdp->deletePrimitive( *newPrim1 );
				//gdp->deletePrimitive( *primNeighbor1 );
			}  // if
			
		}  // for i
		
		
		
		
		
		
		/*
		// Get all the geometry points, which are the starting points from which the fractal growth will occur
        GEO_PointList& tmp_pts = gdp->points();
        
        // Make a copy of the original pts list so that we can append our added fractal points in the order we want
        int nump = tmp_pts.entries();
        //GEO_PointList pts;
		GEO_PointList *pts = new GEO_PointList();
		//GA_GBElementListT<GEO_Point> pts();
        for ( int i = 0; i < nump; i++ )
        {
            pts.append( tmp_pts.entry(i) );
        }
        
        // Get the current list of point groups.
        //   For this fractal growth to work, each group should contain a pair of neighboring points.
        //   All groups make up every pair of neighboring points.
        //GB_GroupList& ptgrp = gdp->pointGroups();
		GA_ElementGroupTable& ptgrp = gdp->pointGroups();
        int nextGroupNum = ptgrp.length();
        
        // For each loop, create a new fractally-grown particle, group it with each of its neighbors,
        //   then get rid of the old group that consisted of its neighbors
        for ( int i = 0; i < numPoints; i++ )
        {
            // Get a list of all the point groups
            GB_GroupList& pointGroups = gdp->pointGroups();
            int numGroups = pointGroups.length();
            
            // If there are still point groups left, do a fractal growth;
            //   Otherwise, exit the loop
            if ( numGroups > 0 )
            {
                // Choose one random group out of the existing point groups
                int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
                int randGroupIndex = randomNumber % numGroups;
                GB_PointGroup* curGroup = (GB_PointGroup*)( ((UT_LinkList&)pointGroups).find(randGroupIndex) );
                
                // Array to keep track of the point positions in the current group
                UT_Vector4Array pointPositions;
                
                // Get the point positions of the pair of points in the selected group
                GEO_Point *ppt;
                GEO_Point *pt0;
                GEO_Point *pt1;
                fpreal ptCount = 0;
                //UT_Vector4 childPos( 0.0, 0.0, 0.0 );
                FOR_ALL_OPT_GROUP_POINTS( gdp, curGroup, ppt )  // Loops through each GEO_Point, ppt, in the chosen point group, curGroup
                {
                    UT_Vector4 p = ppt->getPos();
                    pointPositions.append( p );
                    
                    if ( ptCount == 0 )
                        pt0 = ppt;
                    else if ( ptCount == 1 )
                        pt1 = ppt;
                    
                    //childPos += p;
                    ptCount += 1;
                }
                
                // Get the parent points' normals.
                //   The normals will determine which direction the new particle should be created in
                //   as well as compute the child particle's normal
                //GB_AttributeRef norm_index = gdp->findPointAttrib( "N", 3 * sizeof(float), GB_ATTRIB_VECTOR );
				GA_RWAttributeRef norm_index = gdp->findFloatTuple( GA_ATTRIB_POINT, "N", 3 );
                //UT_Vector3* n0 = pt0->castAttribData<UT_Vector3>( norm_index );
				//UT_Vector3* n0 = pt0->getValue<UT_Vector3>( norm_index );
                //UT_Vector4 norm0( n0->x(), n0->y(), n0->z() );
				float x0 = pt0->getValue<float>( norm_index, 0 );
				float y0 = pt0->getValue<float>( norm_index, 1 );
				float z0 = pt0->getValue<float>( norm_index, 2 );
				UT_Vector4 norm0( x0, y0, z0 );
                
				//UT_Vector3* n1 = pt1->castAttribData<UT_Vector3>( norm_index );
				//UT_Vector3* n1 = pt1->getValue<UT_Vector3>( norm_index );
                //UT_Vector4 norm1( n1->x(), n1->y(), n1->z() );
				float x1 = pt1->getValue<float>( norm_index, 0 );
				float y1 = pt1->getValue<float>( norm_index, 1 );
				float z1 = pt1->getValue<float>( norm_index, 2 );
				UT_Vector4 norm1( x1, y1, z1 );
                
                // Compute where the fractally grown (child) particle position should be,
                //   based off the position of its two parent particles (which came from the randomly selected group).
                UT_Vector4 childPos = computeChildPosition( pointPositions[0], pointPositions[1], norm0, 1 );
                
                // Create a point at the child's position
                GEO_Point* newPt = gdp->appendPoint();
                newPt->getPos() = childPos;
                
                // Create a sphere at the child's position, to represent its radius
                GU_PrimSphereParms sphParms;
                sphParms.gdp = gdp;
                sphParms.ppt = newPt;
                GU_PrimSphere* newSphere = (GU_PrimSphere*)GU_PrimSphere::build( sphParms );
                
                UT_Matrix4 sphTrans;
                newSphere->getTransform4( sphTrans );
                sphTrans.scale( sphRad, sphRad, sphRad );
                newSphere->setTransform4( sphTrans );
                
                
                UT_Vector4 pos0 = pt0->getPos();
                UT_Vector4 pos1 = pt1->getPos();
                UT_Vector4 posNew = newPt->getPos();
                
                // Average the points' normals (to be assigned to the fractally grown particle)
                UT_Vector4 avgNorm( (norm0.x()+norm1.x())/2.0, (norm0.y()+norm1.y())/2.0, (norm0.z()+norm1.z())/2.0, 1 );
                //newPt->castAttribData<UT_Vector3>( norm_index )->x() = avgNorm.x();  //(norm0.x()+norm1.x())/2.0;
                //newPt->castAttribData<UT_Vector3>( norm_index )->y() = avgNorm.y();  //(norm0.y()+norm1.y())/2.0;
                //newPt->castAttribData<UT_Vector3>( norm_index )->z() = avgNorm.z();  //(norm0.z()+norm1.z())/2.0;
				newPt->setValue<float>( norm_index, avgNorm.x(), 0 );
				newPt->setValue<float>( norm_index, avgNorm.y(), 1 );
				newPt->setValue<float>( norm_index, avgNorm.z(), 2 );
                
                // Set up data for computing which particles to group the new child particle with
                int numPts = pts.entries();
                //int minDist = 4 * sphRad * sphRad;    // (2r)^2 is the minimum squared distance that the new sphere's groups' particles can be from any other sphere in the wavefront
                int minDist = 2 * sphRad;               // (2r) is the minimum squared distance that the new sphere's groups' particles can be from any other sphere in the wavefront
                
                // Figure out which direction to go around the ring
                int dir = 0;
                //UT_Vector4 pt0pt1 = pt0->getPos() - pt1->getPos();
                //if ( cross( pt0pt1, avgNorm ).y() < 0 )
                //    dir = 1;
                //else
                //    dir = -1;
                if ( pts.find( pt0 ) > pts.find( pt1 ) && !( pts.find( pt1 ) == 0 && pts.find( pt0 ) == numPts-1 ) 
                        || ( pts.find( pt0 ) == 0 && pts.find( pt1 ) == numPts-1 ) )
                    dir = 1;
                else
                    dir = -1;
                
                if ( doContinue )
                {
                    gdp->deletePoint( newPt->getNum() );
                    continue;
                }
                
                
                gdp->destroyPointGroup( curGroup );
                
                
                int iNext0 = ( pts.find( pt0 ) + dir + numPts ) % numPts;
                for ( int n = 0; n < numPts-2; n++, iNext0 = (iNext0+dir+numPts)%numPts )
                {
                    GEO_Point* nextPt0 = pts.entry( iNext0 );
                    UT_Vector4 nextPos0 = nextPt0->getPos();
                    UT_Vector4 v0New = newPos - pos0;
                    UT_Vector4 v0Next = nextPos0 - pos0;
                    if ( v0New.dot( v0Next ) > -0.5 && norm0.dot( v0New ) > 0 )
                    {
                        // Get rid of groups containing pt0
                        GB_GroupList& pointGroups0 = gdp->pointGroups();
                        GB_Group *curr = pointGroups0.head();
                        while( curr )
                        {
                            GB_Group *tmp = curr;
                            curr = (GB_Group*)curr->next();
                            
                            if ( tmp->contains( pt0 ) )
                            {
                                gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                            }  // if
                        }  // while
                        
                        // Remove pt0 from the wavefront
                        pts.remove( pt0 );
                        
                        pt0 = nextPt0;
                        pos0 = pt0->getPos();
                        numPts = pts.entries();
                        iNext0 = pts.find( pt0 );
                    }  // if
                    else
                    {
                        break;
                    }
                }  // for n
                
                
                int iNext1 = ( pts.find( pt1 ) - dir + numPts ) % numPts;
                for ( int n = 0; n < numPts-2; n++, iNext1 = (iNext1-dir+numPts)%numPts )
                {
                    GEO_Point* nextPt1 = pts.entry( iNext1 );
                    UT_Vector4 nextPos1 = nextPt1->getPos();
                    UT_Vector4 v1New = newPos - pos1;
                    UT_Vector4 v1Next = nextPos1 - pos1;
                    if ( v1New.dot( v1Next ) > -0.5 && norm1.dot( v1New ) > 0 )
                    {
                        // Get rid of groups containing pt0
                        GB_GroupList& pointGroups0 = gdp->pointGroups();
                        GB_Group *curr = pointGroups0.head();
                        while( curr )
                        {
                            GB_Group *tmp = curr;
                            curr = (GB_Group*)curr->next();
                            
                            if ( tmp->contains( pt1 ) )
                            {
                                gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                            }  // if
                        }  // while
                        
                        // Remove pt0 from the wavefront
                        pts.remove( pt1 );
                        
                        pt1 = nextPt1;
                        pos1 = pt1->getPos();
                        numPts = pts.entries();
                        iNext1 = pts.find( pt1 );
                    }  // if
                    else
                    {
                        break;
                    }
                }  // for n
                
                
                //if ( pt0->getNum() == pt1->getNum() )
                //    cout << "   YIKES: " << pt0->getNum() << " == " << pt1->getNum() << "!!!!!" << endl;
                
                // Set up new group with pt0
                UT_String newGroupName( "group_" );
                int newGroupNum = nextGroupNum++;          // A new group is being added, so the number of groups increments
                char numstr[UT_NUMBUF];
                UT_String::itoa( numstr, newGroupNum );
                newGroupName += numstr;
                GB_PointGroup* newGroup = gdp->newPointGroup( newGroupName );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                //cout << "   created a group" << endl;
                newGroup->add( pt0 );
                newGroup->add( newPt );
                
                // Set up new group with pt1
                UT_String newGroupName1( "group_" );
                int newGroupNum1 = nextGroupNum++;          // A new group is being added, so the number of groups increments
                char numstr1[UT_NUMBUF];
                UT_String::itoa( numstr1, newGroupNum1 );
                newGroupName1 += numstr1;
                GB_PointGroup* newGroup1 = gdp->newPointGroup( newGroupName1 );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                //cout << "   created a group" << endl;
                newGroup1->add( newPt );
                newGroup1->add( pt1 );
                
                // Insert the new point in order into the array of points
                int newPtIndex = -1;
                if ( dir == -1 )
                {
                    newPtIndex = pts.find( pt0 );
                    pts.insert( newPt, (newPtIndex-dir+numPts)%numPts );
                }
                else
                {
                    newPtIndex = pts.find( pt0 );
                    pts.insert( newPt, (newPtIndex+numPts)%numPts );
                }
                numPts = pts.entries();
                
                //cout << "  inputting point " << newPt->getNum() << endl;
                
            }  // if
            else
            {
                break;
                
            }  // else
        }  // for
		*/
		
    }  // if
	
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    
    return error();
}
