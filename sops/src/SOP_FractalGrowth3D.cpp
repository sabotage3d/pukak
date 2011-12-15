#include "SOP_FractalGrowth3D.h"


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
SOP_FractalGrowth3D::myTemplateList[] = {
    PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0, &PRMscaleRange),
    PRM_Template(PRM_INT_J,	1, &names[1], &defEighteen, 0, &defNumParticlesRange),
	PRM_Template(PRM_INT_J,	1, &names[2], &defFive, 0, &defSeedRange),
    PRM_Template(),
};




void
newSopOperator( OP_OperatorTable *table )
{
     table->addOperator(new OP_Operator("fractalgrowth3d",
					"Fractal Growth 3D",
					 SOP_FractalGrowth3D::myConstructor,
					 SOP_FractalGrowth3D::myTemplateList,
					 1,
					 1,
					 0));
}




OP_Node *
SOP_FractalGrowth3D::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_FractalGrowth3D(net, name, op);
}




SOP_FractalGrowth3D::SOP_FractalGrowth3D( OP_Network *net, const char *name, OP_Operator *op )
	: SOP_Node(net, name, op), myGroup(0)
{
}

SOP_FractalGrowth3D::~SOP_FractalGrowth3D() {}




UT_Vector3 SOP_FractalGrowth3D::computeChildPosition( UT_Vector4 vertex, UT_Vector4 p0, UT_Vector4 p1, float R )	// Takes in three points of the triangle prim and the radius of our spheres
{
	UT_Vector4 mid0 = ( p0 + vertex ) / 2.0;
	UT_Vector4 mid1 = ( p1 + vertex ) / 2.0;
	
	// Compute the vectors pointing from the vertex to the edge midpoints
	UT_Vector4 v0 = p0 - vertex;
	UT_Vector4 v1 = p1 - vertex;
	v0.normalize();
	v1.normalize();
	
	// Compute the vector perpendicular to the triangle
	UT_Vector3 vPerp = cross( v0, v1 );
	
	// Compute the perpendicular bisectors of the two edges
	//   This is done by crossing each edge vector with the perpendicular vector
	UT_Vector3 bisector0 = cross( vPerp, v0 );
	UT_Vector3 bisector1 = cross( v1, vPerp );
	
	// Now compute the time t of the line equation where the two bisectors intersect
	UT_Vector4 p0p1 = p1 - p0;
	float t = ( (p0p1[0]*bisector1[2] - p0p1[2]*bisector1[0]) / (bisector0[0]*bisector1[2] - bisector0[2]*bisector1[0]) );
	cout << "t = " << t << endl;
	// Computer the circumcenter
	UT_Vector3 circumcenterPos = (UT_Vector3)p0 + bisector0*t;
	
	// Compute the distance between the vertex and the circumcenter
	UT_Vector3 vertexToCircumcenter = circumcenterPos - (UT_Vector3)vertex;
	float bSqr = vertexToCircumcenter.length2();		// The length squared
	
	//         |\
	//         | \
	//         |  \ (2R)^2
	//  height |   \
	//         |    \
	//         |     \
	//          ------ 
	//             b
	// Compute the height of the new sphere's position
	cout << "R = " << R << endl;
	float hypSqr = 4 * R * R;
	cout << "hypSqr = " << hypSqr << endl;
	float height = sqrt( hypSqr - bSqr );
	cout << "height = " << height << endl;
	// Calculate the position of the new sphere
	UT_Vector3 newSpherePos = UT_Vector3( circumcenterPos[0], height, circumcenterPos[2] );
	
	return newSpherePos;
}  // computeChildPosition()
/*
( UT_Vector4 p1, UT_Vector4 p2, UT_Vector4 norm, fpreal radius )
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
*/



//bool SOP_FractalGrowth3D::intersectRaySphere( UT_Vector4 rayOrigin, UT_Vector4 ray, UT_Vector4 sphCenter, fpreal radius )
//{
//    ray.normalize();
//    
//    UT_Vector4 sphVector = sphCenter - rayOrigin;
//    
//    fpreal distFromSphCenterToRay = ( ray.dot(sphVector) * ray - sphVector ).length();
//    
//    if ( distFromSphCenterToRay < radius )
//        return true;
//    else
//        return false;
//
//}  // intersectRaySphere




float SOP_FractalGrowth3D::thresholdAngle( float dist0, float dist1, float radius )
{
	float cDist0 = dist0 < 3.4641 ? dist0 : 3.4641;
	float cDist1 = dist1 < 3.4641 ? dist1 : 3.4641;
	 // Given the distance between p1 and neighP, figure out the angle between neighP-p0-newP
	 //   will allow a particle between newP and p1
	 //   We use eqn: acos((dist+2R)/4R) to figure out what the angle can be between them where the numerator is opp and the denominator is hyp
	 float val0 = acos( (cDist0 / (4.0*radius)) );		// dist0 = distance between the centers of neighP and p0 (includes the two radii of the spheres plus the space between the spheres)
	 float val1 = acos( (cDist1 / (4.0*radius)) );		// dist1 = distance between the centers of p0 and newP
	 float val = val0 - (1.04719755-val1);				// 1.04719755 radians = 60 degrees
	 float threshold = ( (0.5235996 - val) / -0.5235996 ) * -0.5;	// Remap to the threshold value
	 
	 return threshold;
}  // thresholdAngle()




float SOP_FractalGrowth3D::thresholdAngle180( float dist0, float dist1, float radius )
{
	float cDist0 = dist0 < 4.0 ? dist0 : 4.0;
	float cDist1 = dist1 < 4.0 ? dist1 : 4.0;
	 // Given the distance between p1 and neighP, figure out the angle between neighP-p0-newP
	 //   will allow a particle between newP and p1
	 //   We use eqn: acos((dist+2R)/4R) to figure out what the angle can be between them where the numerator is opp and the denominator is hyp
	 float val0 = acos( (cDist0 / (4.0*radius)) );		// dist0 = distance between the centers of neighP and p0 (includes the two radii of the spheres plus the space between the spheres)
	 float val1 = acos( (cDist1 / (4.0*radius)) );		// dist1 = distance between the centers of p0 and newP
	 float val = 1.04719755 + val0 - (1.04719755-val1);				// 1.04719755 radians = 60 degrees
	 float threshold = ( (0.5235996 - val) / -0.5235996 ) * -0.5;	// Remap to the threshold value
	 
	 return threshold;
}  // thresholdAngle180()




OP_ERROR SOP_FractalGrowth3D::cookMySop( OP_Context &context )
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
		for ( int i = 0; i < numSpheresToPopulate; i++ )
        {
			GEO_Primitive* prim = NULL;
			
			GA_RWAttributeRef numintermediatepts_index = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "numIntermediatePts", 1 );
			
			// Get the number of prims
			GEO_PrimList& prims = gdp->primitives();
			int numPrims = prims.entries();
			
			// Pick a random prim
			//int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
			//int randPrimIndex = randomNumber % numPrims;
			//prim = prims(randPrimIndex);
			
			prim = prims(0);
			
			if ( !prim )
			{
				cout << "ERROR on particle " << i << "!  prim not assigned." << endl;
				continue;
			}  // if
			
			// Get the prim's three	vertices
			GEO_Vertex vert0 = prim->getVertexElement(0);
			GEO_Vertex vert1 = prim->getVertexElement(1);
			GEO_Vertex vert2 = prim->getVertexElement(2);
			
			// Get its normal
			GEO_AttributeHandle normAttrib = gdp->getPrimAttribute( "N" );
			normAttrib.setElement( prim );
			UT_Vector4 edgeNormal = normAttrib.getV4();
			
			// Get the prim's points
			GEO_Point* pt0 = vert0.getPt();
			GEO_Point* pt1 = vert1.getPt();
			GEO_Point* pt2 = vert2.getPt();
			UT_Vector3 pt0Pos = pt0->getPos3();
			UT_Vector3 pt1Pos = pt1->getPos3();
			UT_Vector3 pt2Pos = pt2->getPos3();
			
			int numIntermediatePts = prim->getValue<int>( numintermediatepts_index );		// Gets the number of intermediate points on the chosen prim
			/*
			// Get the intermediate point of the current prim, if it exists, otherwise just use pt1
			float x1, y1, z1;
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
			*/
			
			// Create the new sphere's point
			UT_Vector3 newPtPos = computeChildPosition( pt0->getPos(), pt1->getPos(), pt2->getPos(), sphRad );
			//UT_Vector3 newPtPos = computeChildPosition( pt0->getPos(), UT_Vector4(x1,y1,z1,1), edgeNormal, 1 );
			GEO_Point* newPt = gdp->appendPointElement();
			newPt->setPos( newPtPos );
			
			return error();
			
			// Create a prim between each of the prim's old points and the newly created point (keep the edge directionality)
			//   Note: The number of intermediate points attribute (numIntermediatePts) defaults to 0, so we don't have to explicitly set it
			GU_PrimPoly* newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
			newPrim0->appendVertex(pt0);
            newPrim0->appendVertex(newPt);
			
			GU_PrimPoly* newPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
			newPrim1->appendVertex(newPt);
            newPrim1->appendVertex(pt1);
			
			// If there was one intermediate point in the old edge (prim), add that intermediate pt to newPrim1
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
				}  // if
			}  // if
			
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
			if ( dotProd0 > 0 )
			{
				int numNeighborIntermediatePts = primNeighbor0->getValue<int>( numintermediatepts_index );
				if ( numNeighborIntermediatePts == 0 )
				{
					int numNewPrim0Int = newPrim0->getValue<int>( numintermediatepts_index );
					if ( numNewPrim0Int == 1 )	// If the newPrim1 has an interior point
					{
						int newPrim0IntPtNum = newPrim0->getValue<int>( intermediateptnum_index1 );
						UT_Vector3 newPrim0IntPos = newPrim0->getValue<UT_Vector3>( intermediateptpos_index1 );
						
						
						/*
						// If neighP--p0--newP is closed
						UT_Vector3 e1 = newPtPos - pt0Pos;
						UT_Vector3 e2 = neighborPos0 - pt0Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd > angle )
						{
							gdp->deletePrimitive( *newPrim0 );
							
							// Make newP -> neighP
							newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
							newPrim0->appendVertex(neighborPt0);
							newPrim0->appendVertex(newPt);
							
							// If neighP--intP--newP is open
							UT_Vector3 e1 = newPtPos - newPrim0IntPos;
							UT_Vector3 e2 = neighborPos0 - newPrim0IntPos;
							float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
							e1.normalize();
							e2.normalize();
							float dotProd = e1.dot( e2 );
							if ( dotProd > angle )
							{
								// Add intP as the intermediate point
								newPrim0->setValue<int>( intermediateptnum_index1, newPrim0IntPtNum );
								newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, newPrim0IntPos );
								newPrim0->setValue<int>( numintermediatepts_index, 1 );
							}  // if
							
							// Create newPrim1's normal
							UT_Vector3 edgeVector0 = newPt->getPos3() - neighborPt0->getPos3();
							edgeVector0.normalize();
							UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
							newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
							newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
							newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
							
							gdp->deletePrimitive( *primNeighbor0 );
						}  // if
						*/
						
						
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
								// Add p0 as the intermediate point
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
					else			// The newPrim0 does not have an interior point, and neighPrim does not either
					{
						UT_Vector3 e1 = newPtPos - pt0Pos;
						UT_Vector3 e2 = neighborPos0 - pt0Pos;
						float angle = thresholdAngle180( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd > angle ) 		// If the angle is less than the threshold
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
							//else
							//{
							//	cout << "PARTICLE #" << i+1 << endl;
							//}  // else
							
							UT_Vector3 edgeVector0 = newPt->getPos3() - neighborPt0->getPos3();
							edgeVector0.normalize();
							UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
							newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
							newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
							newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
							
							gdp->deletePrimitive( *primNeighbor0 );
						}  // if
					}  // else
				}  // if
				else if ( numNeighborIntermediatePts == 1 )
				{
					// BORROW FROM THE SAME METHOD AS NEIGHMID=0 NEWMID=1
					UT_Vector3 newPrim0IntPos = primNeighbor0->getValue<UT_Vector3>( intermediateptpos_index1 );
					int newPrim0IntPtNum = primNeighbor0->getValue<int>( intermediateptnum_index1 );
					
					/*
					UT_Vector3 e1 = newPtPos - pt0Pos;
					UT_Vector3 e2 = neighborPos0 - pt0Pos;
					float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
					e1.normalize();
					e2.normalize();
					float dotProd = e1.dot( e2 );
					if ( dotProd > angle )			// If the angle is LESS than threshold angle (because the smaller the dot product, the larger the angle)
					{
						gdp->deletePrimitive( *newPrim0 );
						
						newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
						newPrim0->appendVertex(neighborPt0);
						newPrim0->appendVertex(newPt);
						
						GEO_Point* intPt = gdp->points()[neighIntermediatePtNum];
						
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
						
						// Add edge normal
						UT_Vector3 edgeVector0 = newPt->getPos3() - neighborPt0->getPos3();
						edgeVector0.normalize();
						UT_Vector3 normal0( -1*edgeVector0[2], 0, edgeVector0[0] );
						newPrim0->setValue<float>( norm_index, normal0.x(), 0 );
						newPrim0->setValue<float>( norm_index, normal0.y(), 1 );
						newPrim0->setValue<float>( norm_index, normal0.z(), 2 );
						
						gdp->deletePrimitive( *primNeighbor0 );
					}  // if
					*/
					
					// If intP--p0--new is open
					UT_Vector3 e1 = newPrim0IntPos - pt0Pos;
					UT_Vector3 e2 = newPtPos - pt0Pos;
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
						
						// If newP--mid--neighP is open
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
						else
						{
							// Else if newP--pt0--neighP is open
							UT_Vector3 e1 = newPtPos - pt0Pos;
							UT_Vector3 e2 = neighborPos0 - pt0Pos;
							float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
							e1.normalize();
							e2.normalize();
							float dotProd = e1.dot( e2 );
							if ( dotProd < angle )
							{
								// Add p0 as the intermediate point
								newPrim0->setValue<int>( intermediateptnum_index1, pt0->getMapIndex() );
								newPrim0->setValue<UT_Vector3>( intermediateptpos_index1, pt0Pos );
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
				}  // else if
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
			if ( dotProd1 > 0 ) //&& edgeLen1 < 3.4641*sphRad )
			{
				int numNeighborIntermediatePts = primNeighbor1->getValue<int>( numintermediatepts_index );
				if ( numNeighborIntermediatePts == 0 )
				{
					int numNewPrim1Int = newPrim1->getValue<int>( numintermediatepts_index );
					if ( numNewPrim1Int == 1 )	// If the newPrim1 has an interior point
					{
						int prim1IntPtNum = newPrim1->getValue<int>( intermediateptnum_index1 );
						UT_Vector3 newPrim1IntPos = newPrim1->getValue<UT_Vector3>( intermediateptpos_index1 );
						
						
						/*
						// If newP--p1--neighP is closed
						UT_Vector3 e1 = newPtPos - pt1Pos;
						UT_Vector3 e2 = neighborPos1 - pt1Pos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd > angle )
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
							
							// Create newPrim1's normal
							UT_Vector3 edgeVector1 = neighborPt1->getPos3() - newPt->getPos3();
							edgeVector1.normalize();
							UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
							newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
							newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
							newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
							
							gdp->deletePrimitive( *primNeighbor1 );
						}  // if
						*/
						
						
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
						UT_Vector3 e1 = newPtPos - pt1Pos;
						UT_Vector3 e2 = neighborPos1 - pt1Pos;
						float angle = thresholdAngle180( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd > angle )			// If the angle is less than the threshold angle
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
							//else
							//{
							//	cout << "PARTICLE #" << i+1 << endl;
							//}  // else
								
							UT_Vector3 edgeVector1 = neighborPt1->getPos3() - newPt->getPos3();
							edgeVector1.normalize();
							UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
							newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
							newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
							newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
							
							gdp->deletePrimitive( *primNeighbor1 );
						}  // if
					}  // else
				}  // if
				else if ( numNeighborIntermediatePts == 1 )
				{
					// BORROW FROM THE SAME METHOD AS NEIGHMID=0 NEWMID=1
					UT_Vector3 newPrim1IntPos = primNeighbor1->getValue<UT_Vector3>( intermediateptpos_index1 );
					float prim1IntPtNum = primNeighbor1->getValue<int>( intermediateptnum_index1 );
					
					/*
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
						
						UT_Vector3 e1 = newPtPos - neighIntermediatePtPos;
						UT_Vector3 e2 = neighborPos1 - neighIntermediatePtPos;
						float angle = thresholdAngle( e1.length(), e2.length(), sphRad );
						e1.normalize();
						e2.normalize();
						float dotProd = e1.dot( e2 );
						if ( dotProd < angle )
						{
							newPrim1->setValue<UT_Vector3>( intermediateptpos_index1, neighIntermediatePtPos );
							newPrim1->setValue<int>( intermediateptnum_index1, neighIntermediatePtNum );
							newPrim1->setValue<int>( numintermediatepts_index, 1 );
						}  // if
						
						// Add edge normal
						UT_Vector3 edgeVector1 = neighborPt1->getPos3() - newPt->getPos3();
						edgeVector1.normalize();
						UT_Vector3 normal1( -1*edgeVector1[2], 0, edgeVector1[0] );
						newPrim1->setValue<float>( norm_index, normal1.x(), 0 );
						newPrim1->setValue<float>( norm_index, normal1.y(), 1 );
						newPrim1->setValue<float>( norm_index, normal1.z(), 2 );
						
						gdp->deletePrimitive( *primNeighbor1 );
					}  // if
					*/
					
					// If intP--p1--newP is open
					UT_Vector3 e1 = newPrim1IntPos - pt1Pos;
					UT_Vector3 e2 = newPtPos - pt1Pos;
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
						
						// If newP--p1--neighP is open
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
						else
						{
							// Else if newP--intP--neighP is open
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
				}  // else if
			}  // if
		}  // for i
    }  // if
	
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    
    return error();
}
