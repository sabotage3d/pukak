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




UT_Vector3 SOP_FractalGrowth3D::cross( UT_Vector3 a, UT_Vector3 b )
{
	float x = a[1]*b[2] - a[2]*b[1];
	float y = a[2]*b[0] - a[0]*b[2];
	float z = a[0]*b[1] - a[1]*b[0];
	return UT_Vector3( x, y, z );
}  // cross()




UT_Vector3 SOP_FractalGrowth3D::computeChildPosition( UT_Vector3 vertex, UT_Vector3 p0, UT_Vector3 p1, UT_Vector3 normal, float R, GU_Detail* gdp, int time )	// Takes in three points of the triangle prim and the radius of our spheres
{
	UT_Vector3 mid0 = ( p0 + vertex ) / 2.0;
	UT_Vector3 mid1 = ( p1 + vertex ) / 2.0;
	
	// Compute the vectors pointing from the vertex to the edge midpoints
	UT_Vector3 v0 = mid0 - vertex;
	UT_Vector3 v1 = mid1 - vertex;
	v0.normalize();
	v1.normalize();
	
	// Compute the vector perpendicular to the triangle
	UT_Vector3 vPerp = cross( v0, v1 );
	
	// Compute the perpendicular bisectors of the two edges
	//   This is done by crossing each edge vector with the perpendicular vector
	//   They are automatically normalized since v0, v1, and vPerp are all normalized
	UT_Vector3 bisector0 = cross( vPerp, v0 );
	UT_Vector3 bisector1 = cross( v1, vPerp );
	
	// Now compute the time t of the line equation where the two bisectors intersect
	UT_Vector3 a = bisector0;
	UT_Vector3 b = bisector1;
	UT_Vector3 c = mid1 - mid0;
	UT_Vector3 d = cross(a,b);
	float t = ( cross(c,b).dot( d ) ) / ( d.length2() );		// http://mathworld.wolfram.com/Line-LineIntersection.html
	//UT_Vector3 p0p1 = mid1 - mid0;
	//if ( cross(bisector0, bisector1).dot( p0p1 ) < 0 )
	//	p0p1 *= -1;
	//float t = ( (p0p1[0]*bisector1[2] - p0p1[2]*bisector1[0]) / (bisector0[0]*bisector1[2] - bisector0[2]*bisector1[0]) );
	
	// Computer the circumcenter
	UT_Vector3 circumcenterPos = mid0 + bisector0*t;
	
	// Compute the distance between the vertex and the circumcenter
	UT_Vector3 vertexToCircumcenter = circumcenterPos - vertex;
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
	float hypSqr = 4 * R * R;
	float height = sqrt( abs(hypSqr - bSqr) );
	
	// Raise the circumcenter point along the height
	normal.normalize();
	circumcenterPos = circumcenterPos + normal*height;
	
	// Calculate the position of the new sphere
	UT_Vector3 newSpherePos = UT_Vector3( circumcenterPos[0], circumcenterPos[1], circumcenterPos[2] );
	
	return newSpherePos;
}  // computeChildPosition()




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




OP_ERROR SOP_FractalGrowth3D::cookMySop( OP_Context &context )
{
    GEO_Point   *ppt;
    
    float sphRad = RADIUS();
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
			return error();
		//{
			//cout << "ERROR on particle " << i << "!  prim not assigned." << endl;
			//continue;
		//}  // if
		
		// Get the prim's three	vertices
		GEO_Vertex vert0 = prim->getVertexElement(0);
		GEO_Vertex vert1 = prim->getVertexElement(1);
		GEO_Vertex vert2 = prim->getVertexElement(2);
		
		// Get its normal
		UT_Vector3 edgeNormal = prim->computeNormal();
		
		// Set the parent prim's normal attribute
		GEO_AttributeHandle normAttrib = gdp->getPrimAttribute( "N" );
		normAttrib.setElement( prim );
		normAttrib.setV3( edgeNormal );
		
		// Get the prim's points
		GEO_Point* pt0 = vert0.getPt();
		GEO_Point* pt1 = vert1.getPt();
		GEO_Point* pt2 = vert2.getPt();
		UT_Vector3 pt0Pos = pt0->getPos3();
		UT_Vector3 pt1Pos = pt1->getPos3();
		UT_Vector3 pt2Pos = pt2->getPos3();
		
		int numIntermediatePts = prim->getValue<int>( numintermediatepts_index );		// Gets the number of intermediate points on the chosen prim
		
		// Create the new sphere's point
		UT_Vector3 newPtPos = computeChildPosition( pt0Pos, pt1Pos, pt2Pos, edgeNormal, sphRad, gdp, (int)(t*24.0) );
		//UT_Vector3 newPtPos = computeChildPosition( pt0->getPos(), UT_Vector4(x1,y1,z1,1), edgeNormal, 1 );
		GEO_Point* newPt = gdp->appendPointElement();
		newPt->setPos( newPtPos );		
    }  // if
	
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    
    return error();
}  // cookMySop()
