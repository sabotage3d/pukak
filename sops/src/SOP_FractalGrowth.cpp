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




#define SEPARATIONDIST		3.988
#define ANGLE				-0.4988




static PRM_Name        names[] = {
    PRM_Name("rad",	"Radius"),
    PRM_Name("numpoints", "Num Points"),
	PRM_Name("seed", "Seed"),
    //PRM_Name("phase",	"Phase"),
    //PRM_Name("period",	"Period"),
};

static PRM_Default         defFive(5);
static PRM_Default         defEighteen(18);
static PRM_Range           defNumParticlesRange( PRM_RANGE_UI, 0, PRM_RANGE_UI, 65 );
static PRM_Range           defSeedRange( PRM_RANGE_UI, 0, PRM_RANGE_UI, 100 );

PRM_Template
SOP_FractalGrowth::myTemplateList[] = {
    PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0, &PRMscaleRange),
    PRM_Template(PRM_INT_J,	1, &names[1], &defEighteen, 0, &defNumParticlesRange),
	PRM_Template(PRM_INT_J,	1, &names[2], &defFive, 0, &defSeedRange),
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




UT_Vector3 SOP_FractalGrowth::computeChildPosition( UT_Vector4 p1, UT_Vector4 p2, UT_Vector4 norm, fpreal radius )
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
        //if ( cross(p2p1,norm).y() < 0 )
		if ( p2p1.dot(norm) < -0.001 )
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
        
    return UT_Vector3( p2p1Norm[0], p2p1Norm[1], p2p1Norm[2] );

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




float SOP_FractalGrowth::thresholdAngle( float dist0, float dist1, float radius )
{
	float cDist0 = dist0 < 3.4641 ? dist0 : 3.4641;
	float cDist1 = dist1 < 3.4641 ? dist1 : 3.4641;
	 // Given the distance between p1 and neighP, figure out the angle between neighP-p0-newP
	 //   will allow a particle between newP and p1
	 //   We use eqn: acos((dist+2R)/4R) to figure out what the angle can be between them where the numerator is opp and the denominator is hyp
	 float val0 = acos( (cDist0 / (4.0*radius)) );		// dist0 = distance between the centers of neighP and p0 (includes the two radii of the spheres plus the space between the spheres)
	 float val1 = acos( (cDist1 / (4.0*radius)) );		// dist1 = distance between the centers of p0 and newP
	 float val = val0 - (1.04719755-val1);
	 float threshold = ( (0.5235996 - val) / -0.5235996 ) * -0.5;	// Remap to the threshold value
	 
	 return threshold;
}  // willSphereFit()




OP_ERROR SOP_FractalGrowth::cookMySop( OP_Context &context )
{
    GEO_Point   *ppt;
    
    int sphRad = RADIUS();
    int numSpheresToPopulate = NUMPOINTS();
	int seed = SEED();
    
    // Before we do anything, we must lock our inputs.  Before returning,
    //	we have to make sure that the inputs get unlocked.
    if ( lockInputs(context) >= UT_ERROR_ABORT )
        return error();
    
    float t = context.myTime;
	
	srand ( seed );
    
    // Duplicate our incoming geometry with the hint that we only
    // altered points.  Thus if we our input was unchanged we can
    // easily roll back our changes by copying point values.
    duplicatePointSource(0, context);
    
    if ( error() < UT_ERROR_ABORT )
    {
		GA_PrimitiveGroup* oneIntermediatePrimGroup = gdp->newPrimitiveGroup( "oneIntermediatePrimGroup" );
		GA_PrimitiveGroup* twoIntermediatesPrimGroup = gdp->newPrimitiveGroup( "twoIntermediatesPrimGroup" );
		for ( int i = 0; i < numSpheresToPopulate; i++ )
        {
			GEO_Primitive* prim = NULL;
			
			// See if there are any prims with 2 intermediate points
			GEO_Primitive* tmpprim;
			GA_RWAttributeRef numintermediatepts_index = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "numIntermediatePts", 1 );
			GA_FOR_ALL_PRIMITIVES(gdp, tmpprim)
			{
				int numIntermediatePts = tmpprim->getValue<int>( numintermediatepts_index );
				if ( numIntermediatePts == 1 )
				{
					oneIntermediatePrimGroup->add( tmpprim );
				}  // if
				if ( numIntermediatePts == 2 )
				{
					twoIntermediatesPrimGroup->add( tmpprim );
					break;
				}  // if
			}  // GA_FOR_ALL_PRIMITIVES
			
			int numPrimsWithOneIntermediate = oneIntermediatePrimGroup->entries();
			int numPrimsWithTwoIntermediates = twoIntermediatesPrimGroup->entries();
			if ( numPrimsWithTwoIntermediates > 0 )
			{cout << "DON'T WANT TO BE HERE" << endl;
				GA_FOR_ALL_GROUP_PRIMITIVES( gdp, twoIntermediatesPrimGroup, tmpprim )
				{
					prim = tmpprim;
					break;
				}  // GA_FOR_ALL_GROUP_PRIMITIVES
			}  // if
			else if ( false )//numPrimsWithOneIntermediate > 0 )
			{
				int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
				int randPrimIndex = randomNumber % numPrimsWithOneIntermediate;
				int count = 0;
				GA_FOR_ALL_GROUP_PRIMITIVES( gdp, oneIntermediatePrimGroup, tmpprim )
				{
					//if ( count == randPrimIndex )
					//{
						prim = tmpprim;
						break;
					//}  // if
					
					//count++;
				}  // GA_FOR_ALL_GROUP_PRIMITIVES
			}  // else if
			else
			{
				// Get the number of prims
				GEO_PrimList& prims = gdp->primitives();
				int numPrims = prims.entries();
				
				// Pick a random prim
				//int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
				//int randPrimIndex = randomNumber % numPrims;
				//prim = prims(randPrimIndex);
				
				prim = prims(0);
			}  // else
			
			oneIntermediatePrimGroup->clear();
			twoIntermediatesPrimGroup->clear();
			
			if ( !prim )
			{
				cout << "ERROR on particle " << i << "!  prim not assigned." << endl;
				continue;
			}  // if
			
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
			UT_Vector3 newPtPos = computeChildPosition( pt0->getPos(), UT_Vector4(x1,y1,z1,1), edgeNormal, 1 );
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
			//cout << "added pts " << pt0->getMapIndex() << " " << newPt->getMapIndex() << " " << pt1->getMapIndex() << " to prims " << newPrim0->getMapIndex() << " " << newPrim1->getMapIndex() << endl;
			
			
			// If there was one intermediate point in the old edge (prim), add that intermediate pt to newPrim1
			bool doContinue = false;
			if ( numIntermediatePts == 1 )
			{
				GA_RWAttributeRef intermediateptnum_index1 = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "intermediatePtNum1", 1 );
				GA_RWAttributeRef intermediateptpos_index1 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt1", 3 );
				
				// Get prim's first intermediate point info
				int intPtNum = prim->getValue<int>( intermediateptnum_index1 );
				UT_Vector3 intPtPos = prim->getValue<UT_Vector3>( intermediateptpos_index1 );
				
				UT_Vector3 newEdge = newPtPos - pt1Pos;
				float newEdgeLength = newEdge.length();
				UT_Vector3 e1 = newPtPos - intPtPos;
				UT_Vector3 e2 = pt1Pos - intPtPos;
				
				float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
				e1.normalize();
				e2.normalize();
				float dotProd = e1.dot( e2 );
				if ( dotProd < angle || newEdgeLength > SEPARATIONDIST*sphRad )		// If the angle is greater than 120 degrees
				{
					// Set newPrim1's first intermediate point info
					newPrim1->setValue<int>( intermediateptnum_index1, intPtNum );
					newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, intPtPos );
					newPrim1->setValue<int>( numintermediatepts_index, 1 );
					doContinue = true;
				}  // if
			}  // if
			
			
			// If there were two intermediate points in the old edge (prim), add the second intermediate point to newPrim1
			/*
			if ( numIntermediatePts == 2 )
			{
				//UT_Vector3 newEdge = newPtPos - pt1Pos;
				//float newEdgeLength = newEdge.length();
				
				//if ( newEdgeLength > SEPARATIONDIST * sphRad )		// If angle is less than 120 degrees
				//{
					GA_RWAttributeRef intermediateptnum_index1 = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "intermediatePtNum1", 1 );
					GA_RWAttributeRef intermediateptpos_index1 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt1", 3 );
					GA_RWAttributeRef intermediateptnum_index2 = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "intermediatePtNum2", 1 );
					GA_RWAttributeRef intermediateptpos_index2 = gdp->findFloatTuple( GA_ATTRIB_PRIMITIVE, "intermediatePt2", 3 );
					
					// Get prim's first intermediate point info
					int intPtNum1 = prim->getValue<int>( intermediateptnum_index1 );
					int intPtNum2 = prim->getValue<int>( intermediateptnum_index2 );
					UT_Vector3 intPtPos1 = prim->getValue<UT_Vector3>( intermediateptpos_index1 );
					UT_Vector3 intPtPos2 = prim->getValue<UT_Vector3>( intermediateptpos_index2 );
					
					// If angle between newPt and int1 > 120, insert the prim's first intermediate point
					// Else if the angle b/t newPt and int2 > 120, insert the prim's second intermediate point
					UT_Vector3 intEdge2 = intPtPos2 - newPtPos;
					float intEdgeLength2 = intEdge2.length();
					UT_Vector3 e1 = newPtPos - intPtPos1;
					UT_Vector3 e2 = intPtPos2 - intPtPos1;
					float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
					e1.normalize();
					e2.normalize();
					float dotProd = e1.dot( e2 );
					if ( dotProd < angle || intEdgeLength2 > SEPARATIONDIST * sphRad)
					{
						// Set newPrim1's first intermediate point info
						newPrim1->setValue<int>( intermediateptnum_index1, intPtNum1 );
						newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, intPtPos1 );
						newPrim1->setValue<int>( numintermediatepts_index, 1 );
						
						UT_Vector3 edge2 = pt1Pos - intPtPos1;
						float edge2Length = edge2.length();
						UT_Vector3 e1 = intPtPos1 - intPtPos2;
						UT_Vector3 e2 = pt1Pos - intPtPos2;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd < angle && edge2Length > SEPARATIONDIST*sphRad )
						{
							// Set newPrim1's second intermediate point info
							newPrim1->setValue<int>( intermediateptnum_index2, intPtNum2 );
							newPrim1->setValue<UT_Vector3>( intermediateptpos_index2, intPtPos2 );
							newPrim1->setValue<int>( numintermediatepts_index, 2 );
						}  // if
					}  // if
					else
					{
						UT_Vector3 edge2 = pt1Pos - newPtPos;
						float edge2Length = edge2.length();
						UT_Vector3 e1 = newPtPos - intPtPos2;
						UT_Vector3 e2 = pt1Pos - intPtPos2;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd < angle || edge2Length > SEPARATIONDIST*sphRad )
						{
							// Set newPrim1's first intermediate point info
							newPrim1->setValue<int>( intermediateptnum_index1, intPtNum2 );
							newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, intPtPos2 );
							newPrim1->setValue<int>( numintermediatepts_index, 1 );
						}  // if
					}  // else
				//}  // if
				//doContinue = true;
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
			float edgeLen0 = edgeVector0.length();
			edgeVector0.normalize();
			UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
			newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
			newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
			newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
			
			UT_Vector3 edgeVector1 = pt1->getPos3() - newPt->getPos3();
			float edgeLen1 = edgeVector1.length();
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
			UT_Vector3 newPtToNeighbor0 = neighborPos0 - newPtPos;
			float dotProd0 = normal0.dot( newPtToNeighbor0 );
			
			// If prim0 has a concavity with its neighbor:
			//    Create a prim between prim0's pt1 and its neighbor's unshared point
			//    If the triangle formed from the filled concavity has an angle greater than 120 degrees
			//       Keep the whole triangle around (all 3 prims)
			if ( dotProd0 > 0 )//&& edgeLen0 < 3.4641*sphRad )
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
				if ( numNeighborIntermediatePts == 0 )
				{
					int numNewPrim0Int = newPrim0->getValue<int>( numintermediatepts_index );
					if ( numNewPrim0Int == 1 )	// If the newPrim1 has an interior point
					{
						int newPrim0IntPtNum = newPrim0->getValue<int>( intermediateptnum_index1 );
						UT_Vector3 newPrim0IntPos = newPrim0->getValue<UT_Vector3>( intermediateptpos_index1 );
						
						// If neighP--p0--intP is open
						UT_Vector3 e1 = newPrim0IntPos - pt0Pos;
						UT_Vector3 e2 = neighborPos0 - pt0Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd < angle )
						{
							// Do nothing, keep the newPrim1 with its interior pt and leave the neighbor alone
						}  // if
						else
						{
							gdp->deletePrimitive( *newPrim0 );
							
							// Make newP -> neighP
							newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
							newPrim0->appendVertex(neighborPt0);
							newPrim0->appendVertex(newPt);
							
							// If newP--p0--neighP is open
							UT_Vector3 e1 = newPtPos - pt0Pos;
							UT_Vector3 e2 = neighborPos0 - pt0Pos;
							float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
							e1.normalize();
							e2.normalize();
							float dotProd = e1.dot( e2 );
							if ( dotProd < angle )
							{
								// Add p1 as the intermediate point
								newPrim0->setValue<int>( intermediateptnum_index1, pt0->getMapIndex() );
								newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, pt0Pos );
								newPrim0->setValue<int>( numintermediatepts_index, 1 );
							}  // if
							else
							{
								// Else if newP--intP--neighP is open
								UT_Vector3 e1 = newPtPos - newPrim0IntPos;
								UT_Vector3 e2 = neighborPos0 - newPrim0IntPos;
								float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
								e1.normalize();
								e2.normalize();
								float dotProd = e1.dot( e2 );
								if ( dotProd < angle )
								{
									// Add intP as the intermediate point
									newPrim0->setValue<int>( intermediateptnum_index1, newPrim0IntPtNum );
									newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, newPrim0IntPos );
									newPrim0->setValue<int>( numintermediatepts_index, 1 );
								}  // if
							}  // else
							
							// Create newPrim1's normal
							UT_Vector3 edgeVector0 = newPt->getPos3() - neighborPt0->getPos3();
							edgeVector0.normalize();
							UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
							newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
							newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
							newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
							
							gdp->deletePrimitive( *primNeighbor0 );
						}  // else
					}  // if
					else			// The newPrim0 does not have an interior point
					{
						gdp->deletePrimitive( *newPrim0 );
						
						newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						newPrim0->appendVertex(neighborPt0);
						newPrim0->appendVertex(newPt);
						
						UT_Vector3 e1 = newPtPos - pt0Pos;
						UT_Vector3 e2 = neighborPos0 - pt0Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd < angle ) 		// If the angle is greater than 120 degrees
						{
							newPrim0->setValue<int>( intermediateptnum_index1, pt0->getMapIndex() );
							newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, pt0Pos );
							newPrim0->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						
						UT_Vector3 edgeVector0 = newPt->getPos3() - neighborPt0->getPos3();
						edgeVector0.normalize();
						UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
						newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
						newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
						newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
						
						gdp->deletePrimitive( *primNeighbor0 );
					}  // else
				}  // if
				else if ( numNeighborIntermediatePts == 1 )
				{
					//if ( doContinue )
					//	continue;
					UT_Vector3 neighIntermediatePtPos = primNeighbor0->getValue<UT_Vector3>( intermediateptpos_index1 );
					int neighIntermediatePtNum = primNeighbor0->getValue<int>( intermediateptnum_index1 );
					
					UT_Vector3 e1 = newPtPos - pt0Pos;
					UT_Vector3 e2 = neighborPos0 - pt0Pos;
					float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
					e1.normalize();
					e2.normalize();
					float dotProd = e1.dot( e2 );
					//float neighEdgeLen = e2.length();
					if ( dotProd > angle )			// If the angle is LESS than threshold angle (because the smaller the dot product, the larger the angle)
					{
						gdp->deletePrimitive( *newPrim0 );
						
						newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						newPrim0->appendVertex(neighborPt0);
						newPrim0->appendVertex(newPt);
						
						GEO_Point* intPt = gdp->points()[neighIntermediatePtNum];
						
						// Get the angle between the neighbor's intermediate point and point0
						//UT_Vector3 neighEdge0 = neighborPos0 - newPtPos;
						//float neighEdge0Len = neighEdge0.length();
						// If neighP--intP--newP is large enough, insert intP as the new intP
						UT_Vector3 e1 = neighborPos0 - neighIntermediatePtPos;
						UT_Vector3 e2 = newPtPos - neighIntermediatePtPos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd < angle )		//|| neighEdge0Len > SEPARATIONDIST * sphRad )
						{
							newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, neighIntermediatePtPos );
							newPrim0->setValue<int>( intermediateptnum_index1, neighIntermediatePtNum );
							newPrim0->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						// Else if intP--pt0--newP is large enough, insert pt0 as the new intP
						else
						{
							UT_Vector3 e1 = neighIntermediatePtPos - pt0Pos;
							UT_Vector3 e2 = newPtPos - pt0Pos;
							float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
							e1.normalize();
							e2.normalize();
							float dotProd = e1.dot( e2 );
							if ( dotProd < angle )		//|| neighEdge0Len > SEPARATIONDIST * sphRad )
							{
								newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, pt0Pos );
								newPrim0->setValue<int>( intermediateptnum_index1, pt0->getMapIndex() );
								newPrim0->setValue<int>( numintermediatepts_index, 1 );
							}  // if
						}  // else
						
						// Add edge normal
						UT_Vector3 edgeVector0 = newPt->getPos3() - neighborPt0->getPos3();
						edgeVector0.normalize();
						UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
						newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
						newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
						newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
						
						gdp->deletePrimitive( *primNeighbor0 );
					}  // if
					else		// See if it is concave with the intermediate point
					{
						UT_Vector3 e1 = newPtPos - pt0Pos;
						UT_Vector3 e2 = neighIntermediatePtPos - pt0Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd > angle )
						{
							// Delete the new prim and the neighbor prim
							gdp->deletePrimitive( *newPrim0 );
							gdp->deletePrimitive( *primNeighbor0 );
							
							GEO_Point* intPt = gdp->points()[neighIntermediatePtNum];
							
							// Create the new prims
							GU_PrimPoly* newPrimNeighbor0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
							newPrimNeighbor0->appendVertex(neighborPt0);
							newPrimNeighbor0->appendVertex(intPt);
							
							newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
							newPrim0->appendVertex(intPt);
							newPrim0->appendVertex(newPt);
							
							// Add edge normals
							UT_Vector3 neighVector0 = intPt->getPos3() - neighborPt0->getPos3();
							neighVector0.normalize();
							UT_Vector3 normal0( -1*neighVector0[2], 0, neighVector0[0] );
							newPrimNeighbor0->setValue<float>( norm_index, normal0.x(), 0 );
							newPrimNeighbor0->setValue<float>( norm_index, normal0.y(), 1 );
							newPrimNeighbor0->setValue<float>( norm_index, normal0.z(), 2 );
							
							UT_Vector3 edgeVector0 = newPt->getPos3() - intPt->getPos3();
							edgeVector0.normalize();
							normal0 = UT_Vector3( -1*edgeVector0[2], 0, edgeVector0[0] );
							newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
							newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
							newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
						}  // if
					}  // else
				}  // else if
			}  // if
			
			
			
			//if ( doContinue )			// If an 1-intermediate prim keeps its intermediate pt.
			//	continue;
			
			
			
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
			if ( dotProd1 > 0 ) //&& edgeLen1 < 3.4641*sphRad )
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
				if ( numNeighborIntermediatePts == 0 )
				{
					int numNewPrim1Int = newPrim1->getValue<int>( numintermediatepts_index );
					if ( numNewPrim1Int == 1 )	// If the newPrim1 has an interior point
					{
						int prim1IntPtNum = newPrim1->getValue<int>( intermediateptnum_index1 );
						UT_Vector3 newPrim1IntPos = newPrim1->getValue<UT_Vector3>( intermediateptpos_index1 );
						
						// If intP--p1--neighP is open
						UT_Vector3 e1 = newPrim1IntPos - pt1Pos;
						UT_Vector3 e2 = neighborPos1 - pt1Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd < angle )
						{
							// Do nothing, keep the newPrim1 with its interior pt and leave the neighbor alone
						}  // if
						else
						{
							gdp->deletePrimitive( *newPrim1 );
							
							// Make newP -> neighP
							newPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
							newPrim1->appendVertex(newPt);
							newPrim1->appendVertex(neighborPt1);
							
							// If newP--intP--neighP is open
							UT_Vector3 e1 = newPtPos - newPrim1IntPos;
							UT_Vector3 e2 = neighborPos1 - newPrim1IntPos;
							float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
							e1.normalize();
							e2.normalize();
							float dotProd = e1.dot( e2 );
							if ( dotProd < angle )
							{
								// Add intP as the intermediate point
								newPrim1->setValue<int>( intermediateptnum_index1, prim1IntPtNum );
								newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, newPrim1IntPos );
								newPrim1->setValue<int>( numintermediatepts_index, 1 );
							}  // if
							else
							{
								// Else if newP--p1--neighP is open
								UT_Vector3 e1 = newPtPos - pt1Pos;
								UT_Vector3 e2 = neighborPos1 - pt1Pos;
								float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
								e1.normalize();
								e2.normalize();
								float dotProd = e1.dot( e2 );
								if ( dotProd < angle )
								{
									// Add p1 as the intermediate point
									newPrim1->setValue<int>( intermediateptnum_index1, pt1->getMapIndex() );
									newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, pt1Pos );
									newPrim1->setValue<int>( numintermediatepts_index, 1 );
								}  // if
							}  // else
							
							// Create newPrim1's normal
							UT_Vector3 edgeVector1 = neighborPt1->getPos3() - newPt->getPos3();
							edgeVector1.normalize();
							UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
							newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
							newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
							newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
							
							gdp->deletePrimitive( *primNeighbor1 );
						}  // else
					}  // if
					else			// The newPrim1 does not have an interior point
					{
						gdp->deletePrimitive( *newPrim1 );
						
						newPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						newPrim1->appendVertex(newPt);
						newPrim1->appendVertex(neighborPt1);
						
						UT_Vector3 e1 = newPtPos - pt1Pos;
						UT_Vector3 e2 = neighborPos1 - pt1Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd < angle )			// If the angle is greater than 120 degrees
						{
							newPrim1->setValue<int>( intermediateptnum_index1, pt1->getMapIndex() );
							newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, pt1Pos );
							newPrim1->setValue<int>( numintermediatepts_index, 1 );
						}  // if
							
						UT_Vector3 edgeVector1 = neighborPt1->getPos3() - newPt->getPos3();
						edgeVector1.normalize();
						UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
						newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
						newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
						newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
						
						gdp->deletePrimitive( *primNeighbor1 );
					}  // else
				}  // if
				else if ( numNeighborIntermediatePts == 1 )
				{
					//if ( doContinue )
					//	continue;
					
					UT_Vector3 neighIntermediatePtPos = primNeighbor1->getValue<UT_Vector3>( intermediateptpos_index1 );
					float neighIntermediatePtNum = primNeighbor1->getValue<int>( intermediateptnum_index1 );
					
					UT_Vector3 e1 = neighborPos1 - pt1Pos;
					UT_Vector3 e2 = newPtPos - pt1Pos;
					float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
					e1.normalize();
					e2.normalize();
					float dotProd = e1.dot( e2 );
					if ( dotProd > angle )		// If the angle is LESS than 120 degrees; neigh1 -- pt1 -- newPt
					{
						gdp->deletePrimitive( *newPrim1 );
						
						newPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						newPrim1->appendVertex(newPt);
						newPrim1->appendVertex(neighborPt1);
						
						// If newP--pt1--intP is large enough, insert pt1 as the new intP
						UT_Vector3 e1 = newPtPos - pt1Pos;
						UT_Vector3 e2 = neighIntermediatePtPos - pt1Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						//if ( dotProd < ANGLE || e1.length() > SEPARATIONDIST*sphRad )			// If the angle is greater than 120 degrees
						if ( dotProd < angle )
						{
							newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, pt1Pos );
							newPrim1->setValue<int>( intermediateptnum_index1, pt1->getMapIndex() );
							newPrim1->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						// Else if newP--intP--neighP is large enough, insert intP as the new intP
						else
						{
							UT_Vector3 e1 = newPtPos - neighIntermediatePtPos;
							UT_Vector3 e2 = neighborPos1 - neighIntermediatePtPos;
							float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
							e1.normalize();
							e2.normalize();
							float dotProd = e1.dot( e2 );
							//if ( dotProd < ANGLE || e1.length() > SEPARATIONDIST*sphRad )			// If the angle is greater than 120 degrees
							if ( dotProd < angle )
							{
								newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, neighIntermediatePtPos );
								newPrim1->setValue<int>( intermediateptnum_index1, neighIntermediatePtNum );
								newPrim1->setValue<int>( numintermediatepts_index, 1 );
							}  // if
						}  // else
					
						// Add edge normal
						UT_Vector3 edgeVector1 = neighborPt1->getPos3() - newPt->getPos3();
						edgeVector1.normalize();
						UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
						newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
						newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
						newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
						
						gdp->deletePrimitive( *primNeighbor1 );
					}  // if
					else		// See if it is concave with the intermediate point
					{
						UT_Vector3 e1 = newPtPos - pt1Pos;
						UT_Vector3 e2 = neighIntermediatePtPos - pt1Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd > angle )
						{
							// Delete the new prim and the neighbor prim
							gdp->deletePrimitive( *newPrim1 );
							gdp->deletePrimitive( *primNeighbor1 );
							
							GEO_Point* intPt = gdp->points()[neighIntermediatePtNum];
							
							// Create the new prims
							newPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
							newPrim1->appendVertex(newPt);
							newPrim1->appendVertex(intPt);
							
							GU_PrimPoly* newPrimNeighbor1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
							newPrimNeighbor1->appendVertex(intPt);
							newPrimNeighbor1->appendVertex(neighborPt1);
							
							// Add edge normals
							UT_Vector3 edgeVector1 = intPt->getPos3() - newPt->getPos3();
							edgeVector1.normalize();
							UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
							newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
							newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
							newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
							
							UT_Vector3 neighVector1 = neighborPt1->getPos3() - intPt->getPos3();
							neighVector1.normalize();
							normal1 = UT_Vector3( -1*neighVector1[2], 0, neighVector1[0] );
							newPrimNeighbor1->setValue<float>( norm_index, normal1.x(), 0 );
							newPrimNeighbor1->setValue<float>( norm_index, normal1.y(), 1 );
							newPrimNeighbor1->setValue<float>( norm_index, normal1.z(), 2 );
						}  // if
					}  // else
				}  // else if
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
                    if ( v0New.dot( v0Next ) > ANGLE && norm0.dot( v0New ) > 0 )
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
                    if ( v1New.dot( v1Next ) > ANGLE && norm1.dot( v1New ) > 0 )
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
