#include "SOP_FractalGrowth.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
//#include <UT/UT_Matrix3.h>
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
	 float val = val0 - (1.04719755-val1);				// 1.04719755 radians = 60 degrees
	 float threshold = ( (0.5235996 - val) / -0.5235996 ) * -0.5;	// Remap to the threshold value
	 
	 return threshold;
}  // thresholdAngle()




float SOP_FractalGrowth::thresholdAngle180( float dist0, float dist1, float radius )
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
		UT_PtrArray<UT_PtrArray<GEO_Point*>> pointGroups;
		
		//GA_PrimitiveGroup* oneIntermediatePrimGroup = gdp->newPrimitiveGroup( "oneIntermediatePrimGroup" );
		//GA_PrimitiveGroup* twoIntermediatesPrimGroup = gdp->newPrimitiveGroup( "twoIntermediatesPrimGroup" );
		for ( int i = 0; i < numSpheresToPopulate; i++ )
        {
			GEO_Primitive* prim = NULL;
			
			// Set up a group to store the current expansion's set of points
			//char tmp[181];
            //sprintf( tmp, "ptGrp%d", i );
            //UT_String curPtGroupName = tmp;
			//GA_PointGroup* curPtGroup = gdp->newPointGroup( curPtGroupName );
			
			// See if there are any prims with 2 intermediate points
			GEO_Primitive* tmpprim;
			GA_RWAttributeRef numintermediatepts_index = gdp->findIntTuple( GA_ATTRIB_PRIMITIVE, "numIntermediatePts", 1 );
			
			// Get the number of prims
			GEO_PrimList& prims = gdp->primitives();
			int numPrims = prims.entries();
			
			// Pick a random prim
			//int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
			//int randPrimIndex = randomNumber % numPrims;
			//prim = prims(randPrimIndex);
			
			prim = prims(0);
			
			//oneIntermediatePrimGroup->clear();
			//twoIntermediatesPrimGroup->clear();
			
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
			//UT_Vector4 childPos = computeChildPosition( pt0->getPos(), pt1->getPos(), edgeNormal, sphRad );
			UT_Vector3 newPtPos = computeChildPosition( pt0->getPos(), UT_Vector4(x1,y1,z1,1), edgeNormal, sphRad );
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
			
			// Add the points to their point group
			UT_PtrArray<GEO_Point*> ptGrp;
			pointGroups.append( ptGrp );
			pointGroups[i].append( pt0 );
			pointGroups[i].append( pt1 );
			pointGroups[i].append( newPt );
			//curPtGroup->add( pt0 );
			//curPtGroup->add( pt1 );
			//curPtGroup->add( newPt );
			
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
		
		/****** FRACTAL GROWTH FINISHED!!! ******/
		
		// Delete all the wavefront prims
		GA_Primitive* prim;
		GA_FOR_ALL_PRIMITIVES( gdp, prim )
		{
			gdp->destroyPrimitive( *prim );
		}  // GA_FOR_ALL_PRIMITIVES
		
		// Create a poly triangle for each point group
		//GA_ElementGroupTable& ptGroups = gdp->pointGroups();
		//UT_PtrArray<GA_ElementGroup*> ptGroupList;
		//ptGroups.getList( ptGroupList );
		//int numGroups = ptGroups.entries();
		int numGroups = pointGroups.entries();
		for ( int i = 0; i < numGroups; i++ )
		{
			//GA_PointGroup* curPtGrp = (GA_PointGroup*)ptGroupList[i];
			
			GU_PrimPoly* newPrim = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
			newPrim->appendVertex( pointGroups[i][0] );
			newPrim->appendVertex( pointGroups[i][1] );
			newPrim->appendVertex( pointGroups[i][2] );
			
			//GA_FOR_ALL_GROUP_POINTS( gdp, curPtGrp, ppt )	// ppt declared way up top
			//{
			//	newPrim->appendVertex( ppt );
			//}  // for all points in the point group
			
			newPrim->close();
			
			//gdp->destroyGroup( curPtGrp );
		}  // for i
    }  // if
	
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    
    return error();
}
