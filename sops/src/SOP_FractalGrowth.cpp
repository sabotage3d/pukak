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

PRM_Template
SOP_FractalGrowth::myTemplateList[] = {
    PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0, &PRMscaleRange),
    PRM_Template(PRM_FLT_J,	1, &names[1], &defEighteen, 0, &PRMscaleRange),
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
{cout << 1 << endl;
    GEO_Point   *ppt;
    
    int sphRad = RADIUS();
    int numSpheresToPopulate = NUMPOINTS();
    
    // Before we do anything, we must lock our inputs.  Before returning,
    //	we have to make sure that the inputs get unlocked.
    if ( lockInputs(context) >= UT_ERROR_ABORT )
        return error();
    
    float t = context.myTime;
    
    // Duplicate our incoming geometry with the hint that we only
    // altered points.  Thus if we our input was unchanged we can
    // easily roll back our changes by copying point values.
    duplicatePointSource(0, context);
    
    if ( error() < UT_ERROR_ABORT )
    {
		for ( int i = 0; i < numSpheresToPopulate; i++ )
        {cout << "i = " << i << endl;
			// Get the number of prims
			GEO_PrimList& prims = gdp->primitives();
			cout << 2 << endl;
			int numPrims = prims.entries();
			cout << "numPrims = " << numPrims << endl;
			// Pick a random prim
			int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
			cout << 3 << endl;
			int randPrimIndex = randomNumber % numPrims;
			cout << 4 << endl;
			GEO_Primitive* prim = prims(randPrimIndex);
			cout << "prim = " << prim << endl;
			cout << "prim num = " << prim->getPrimitiveId() << endl;
			cout << 5 << endl;
			// Get its two vertices
			GEO_Vertex vert0 = prim->getVertexElement(0);
			cout << 'a' << endl;
			GEO_Vertex vert1 = prim->getVertexElement(1);
			cout << 'b' << endl;
			cout << vert0.getPos3() << endl;
			cout << vert1.getPos3() << endl;
			cout << 6 << endl;
			// Get its normal
			GEO_AttributeHandle normAttrib = gdp->getPrimAttribute( "edgeNormal" );
			cout << 7 << endl;
			normAttrib.setElement( prim );
			UT_Vector4 edgeNormal = normAttrib.getV4();
			cout << 8 << endl;
			// Get the prim's points
			GEO_Point* pt0 = vert0.getPt();
			GEO_Point* pt1 = vert1.getPt();
			cout << 9 << endl;
			// Create the new sphere's point
			UT_Vector4 childPos = computeChildPosition( vert0.getPos(), vert1.getPos(), edgeNormal, 1 );
			GEO_Point* newPt = gdp->appendPointElement();
			newPt->setPos( childPos );
			cout << 10 << endl;
			// Delete the prim
			gdp->deletePrimitive( *prim );
			cout << 11 << endl;
			// Create a prim between each of the prim's old points and the newly created point (keep the edge directionality)
			GU_PrimPoly* newPrim0 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
			newPrim0->appendVertex(pt0);
            newPrim0->appendVertex(newPt);
			GU_PrimPoly* newPrim1 = (GU_PrimPoly*)gdp->appendPrimitive( GEO_PRIMPOLY );
			newPrim0->appendVertex(newPt);
            newPrim0->appendVertex(pt1);
			cout << 12 << endl;
			// Get the prim's neighboring prims
			GA_OffsetArray connectedPrims;
			GEO_Primitive* primNeighbor0;
			GEO_Primitive* primNeighbor1;
			// THERE'S A SIMPLER WAY TO DO THIS IF WE KNOW THE ORDERING THAT GETPRIMITIVESREFERENCINGPOINT IS RETURNING THE PRIMS
			gdp->getPrimitivesReferencingPoint( connectedPrims, gdp->pointOffset(pt0->getMapIndex()) );
			if ( gdp->primitiveIndex(connectedPrims[0]) == prim->getMapIndex() )
				primNeighbor0 = gdp->primitives()[gdp->primitiveIndex(connectedPrims[1])];
			else
				primNeighbor0 = gdp->primitives()[gdp->primitiveIndex(connectedPrims[0])];
				
			gdp->getPrimitivesReferencingPoint( connectedPrims, gdp->pointOffset(pt1->getMapIndex()) );
			if ( gdp->primitiveIndex(connectedPrims[0]) == prim->getMapIndex() )
				primNeighbor1 = gdp->primitives()[gdp->primitiveIndex(connectedPrims[1])];
			else
				primNeighbor1 = gdp->primitives()[gdp->primitiveIndex(connectedPrims[0])];
			cout << 13 << endl;
			// If the left prim has a concavity with its left prim neighbor:
			//    Create a prim between the left prim's right point and the left prim neighbor's right point
			//    If the triangle formed from the filled concavity has an angle greater than 120 degrees
			//       Keep the whole triangle around (all 3 prims)
			// If the right prim has a concavity with its right prim neighbor:
			//    Create a prim between the right prim neighbor's right point and the right prim's left point
			//    If the triangle formed from the filled concavity has an angle greater than 120 degrees
			//       Keep the whole triangle around (all 3 prims)
			break;
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
