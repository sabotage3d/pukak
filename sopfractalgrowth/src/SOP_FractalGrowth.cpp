#include "SOP_FractalGrowth.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
//#include <UT/UT_Matrix3.h>
//#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
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
{
    GEO_Point   *ppt;
    
    int sphRad = RADIUS();
    int numPoints = NUMPOINTS();
    
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
        // Get all the geometry points, which are the starting points from which the fractal growth will occur
        GEO_PointList& tmp_pts = gdp->points();
        
        // Make a copy of the original pts list so that we can append our added fractal points in the order we want
        int nump = tmp_pts.entries();
        GEO_PointList pts;
        for ( int i = 0; i < nump; i++ )
        {
            pts.append( tmp_pts.entry(i) );
        }
        
        // Get the current list of point groups.
        //   For this fractal growth to work, each group should contain a pair of neighboring points.
        //   All groups make up every pair of neighboring points.
        GB_GroupList& ptgrp = gdp->pointGroups();
        int nextGroupNum = ptgrp.length();
        
        // For each loop, create a new fractally-grown particle, group it with each of its neighbors,
        //   then get rid of the old group that consisted of its neighbors
        for ( int i = 0; i < numPoints; i++ )
        {
            // Get a list of all the point groups
            GB_GroupList& pointGroups = gdp->pointGroups();
            int numGroups = pointGroups.length();
            /*cout << "numgroups = " << numGroups << endl;
            
            cout << "  ";
            for ( int j = 0; j < numGroups; j++ )
            {
                GEO_Point *ppt;
                GB_PointGroup* grp = (GB_PointGroup*)( ((UT_LinkList&)pointGroups).find(j) );
                FOR_ALL_OPT_GROUP_POINTS( gdp, grp, ppt )
                {
                    cout << ppt->getNum() << ",";
                }
                cout << " ";
            }
            cout << endl;*/
            
            // If there are still point groups left, do a fractal growth;
            //   Otherwise, exit the loop
            if ( numGroups > 0 )
            {
                // Choose one random group out of the existing point groups
                int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
                int randGroupIndex = randomNumber % numGroups;
                GB_PointGroup* curGroup = (GB_PointGroup*)( ((UT_LinkList&)pointGroups).find(randGroupIndex) );
                
                //cout << "   Cur group = " << " ";
                /*GEO_Point *bob;
                FOR_ALL_OPT_GROUP_POINTS( gdp, curGroup, bob )
                {
                    cout << bob->getNum() << "  ";
                }
                cout << endl;*/
                
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
                GB_AttributeRef norm_index = gdp->findPointAttrib( "N", 3 * sizeof(float), GB_ATTRIB_VECTOR );
                UT_Vector3* n0 = pt0->castAttribData<UT_Vector3>( norm_index );
                UT_Vector4 norm0( n0->x(), n0->y(), n0->z() );
                UT_Vector3* n1 = pt1->castAttribData<UT_Vector3>( norm_index );
                UT_Vector4 norm1( n1->x(), n1->y(), n1->z() );
                
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
                
                /*
                // Fix pt0's or pt1's normal if it's not pointing outward (it may be pointing back inward against the direction of fractal growth)
                UT_Vector4 p0new = posNew - pos0;       // Vector from the center of pt0 to the center of newPt
                UT_Vector4 p1new = posNew - pos1;       // Vector from the center of pt1 to the center of newPt
                UT_Vector3 P0xP1 = cross( p0new, p1new );
                UT_Vector3 P0xN0 = cross( p0new, norm0 );
                UT_Vector3 P1xN1 = cross( p1new, norm1 );
                int signP0xP1y = (P0xP1.y() > 0) - (P0xP1.y() < 0);
                int signP0xN0y = (P0xN0.y() > 0) - (P0xN0.y() < 0);
                int signP1xN1y = (P1xN1.y() > 0) - (P1xN1.y() < 0);
                if ( signP0xP1y != signP0xN0y )     // For P0xP1 and P0xN0, the signs of the cross products' y-values should be the same, otherwise flip pt0's normal
                {
                    // Flip norm0 around p0new
                    p0new.normalize();
                    norm0 = 2 * norm0.dot(p0new) * p0new - norm0;
                    
                    // Assign the new norm0 to pt0
                    pt0->castAttribData<UT_Vector3>( norm_index )->x() = norm0.x();
                    pt0->castAttribData<UT_Vector3>( norm_index )->y() = norm0.y();
                    pt0->castAttribData<UT_Vector3>( norm_index )->z() = norm0.z();
                }  // if
                if ( signP0xP1y == signP1xN1y )     // For P0xP1 and P1xN1, the signs of the cross products' y-values should be the opposite, otherwise flip pt1's normal
                {
                    // Flip norm1 around p0new
                    p1new.normalize();
                    norm1 = 2 * norm1.dot(p1new) * p1new - norm1;
                    
                    // Assign the new norm1 to pt1
                    pt1->castAttribData<UT_Vector3>( norm_index )->x() = norm1.x();
                    pt1->castAttribData<UT_Vector3>( norm_index )->y() = norm1.y();
                    pt1->castAttribData<UT_Vector3>( norm_index )->z() = norm1.z();
                }  // if
                */
                
                // Average the points' normals (to be assigned to the fractally grown particle)
                UT_Vector4 avgNorm( (norm0.x()+norm1.x())/2.0, (norm0.y()+norm1.y())/2.0, (norm0.z()+norm1.z())/2.0, 1 );
                newPt->castAttribData<UT_Vector3>( norm_index )->x() = avgNorm.x();  //(norm0.x()+norm1.x())/2.0;
                newPt->castAttribData<UT_Vector3>( norm_index )->y() = avgNorm.y();  //(norm0.y()+norm1.y())/2.0;
                newPt->castAttribData<UT_Vector3>( norm_index )->z() = avgNorm.z();  //(norm0.z()+norm1.z())/2.0;
                
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
                
                //cout << "   pts = ";
                //for ( int p = 0; p < numPts; p++ )
                //    cout << pts[p]->getNum() << " ";
                //cout << endl;
                
                UT_Vector4 newPos = newPt->getPos();    // Position of the child point
                
                bool doContinue = false;
                int indexPt0 = (pts.find( pt0 ) + dir + numPts) % numPts;
                for ( int c = 0; c < numPts-2; c++, indexPt0 = (indexPt0+dir+numPts)%numPts )
                {
                    GEO_Point* curPt = pts.entry( indexPt0 );
                    UT_Vector4 curPos = curPt->getPos();
                    
                    fpreal dist = ( newPos - curPos ).length();
                    
                    if ( dist < minDist )
                    {
                        //cout << "   Collided with " << curPt->getNum() << endl;
                        //cout << "      " << dist << " < " << minDist << endl;
                        //cout << "      destroying group" << endl;
                        gdp->destroyPointGroup( curGroup );
                        
                        UT_Vector4 pos0 = pt0->getPos();
                        UT_Vector4 pos1 = pt1->getPos();
                        
                        int j0 = pts.find( pt0 );
                        int j1 = pts.find( pt1 );
                        int jC = pts.find( curPt );
                        int ct0 = 0;
                        int ct1 = 0;
                        while( j0 != jC )
                        {
                            j0 = ( j0 + dir + numPts ) % numPts;
                            ct0++;
                        }  // while
                        while( j1 != jC )
                        {
                            j1 = ( j1 - dir + numPts ) % numPts;
                            ct1++;
                        }  // while
                        //cout << "   ct0 ct1 = " << ct0 << " " << ct1 << endl;
                        //int rand01 = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
                        //rand01 = rand01 % 2;
                        //if ( pt0->getNum() < pt1->getNum() )   // Get rid of pt0 if it is the older point (has a lower point number) - else, pt1 will be rid of (see else below)
                        if ( ct0 < ct1 )      // Get rid of pt0 if it is the close point to the collision sphere (curPt)
                        {
                            //cout << "      deleting pt0 #" << pt0->getNum() << " ";
                            //GEO_PointList ptsToDelete;
                            int i0 = pts.find( pt0 );
                            int iNext0 = ( i0 + dir + numPts ) % numPts;
                            GEO_Point* pt0Next = pts.entry( iNext0 );
                            UT_Vector4 nextPos0 = pt0Next->getPos();
                            
                            /*
                            // Find iNext, the next neighbor to pt0 that will not potentially intersect the point we're removing from the wavefront
                            for ( int n = 0; n < numPts-2; n++, iNext = (iNext+dir+numPts)%numPts )
                            {
                                //nextPt = pts.entry( iNext );
                                //UT_Vector4 nextPos = nextPt->getPos();
                                //dist0N = ( nextPos - pos0 ).length();
                                //dist1N = ( nextPos - pos0 ).length();
                                //if ( dist0N
                                
                                nextPt = pts.entry( iNext );
                                nextPos = nextPt->getPos();
                                UT_Vector4 tmpPos = computeChildPosition( pos1, nextPos, norm0, sphRad );
                                fpreal tmpDist = ( pos0 - tmpPos ).length();
                                if ( tmpDist >= 2 * sphRad )
                                    break;
                                
                                //nextPt = pts.entry( iNext );
                                //if ( pt0->getNum() < nextPt->getNum() )     // pt0 will not be replaced by an older point
                                //    break;
                                
                                // If the point to replace pt0 in the group (nextPt) is older than pt0, it cannot replace pt0 and must be deleted with all its groups
                                GB_GroupList& pointGroups0 = gdp->pointGroups();
                                GB_Group *curr = pointGroups0.head();
                                while( curr )
                                {
                                    GB_Group *tmp = curr;
                                    curr = (GB_Group*)curr->next();
                                    
                                    if ( tmp->contains( nextPt ) )
                                    {
                                        gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                                    }  // if
                                }  // while
                                
                                pts.remove( nextPt );
                                numPts = pts.entries();
                                
                            }  // for n
                            */
                            
                            // Get rid of pt0's groups
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
                            
                            // Get rid of pt0
                            pts.remove( pt0 );
                            numPts = pts.entries();
                            
                            // If the angle between vectors pt0Next->pt1 and pt0Next->pt0NextNext is less than 120 degrees, replace pt0Next with pt0NextNext
                            iNext0 = pts.find( pt0Next );
                            int iNextNext0 = ( iNext0 + dir + numPts ) % numPts;
                            for ( int n = 0; n < numPts-2; n++, iNextNext0 = (iNextNext0+dir+numPts)%numPts )
                            {
                                GEO_Point* pt0NextNext = pts.entry( iNextNext0 );
                                UT_Vector4 nextNextPos0 = pt0NextNext->getPos();
                                UT_Vector4 vN01 = pos1 - nextPos0;
                                UT_Vector4 vN0NN0 = nextNextPos0 - nextPos0;
                                if ( vN01.dot( vN0NN0 ) > -0.5 ) //&& norm0.dot( vN01 ) > 0 )
                                {
                                    // Get rid of pt0's groups
                                    GB_GroupList& pointGroups0 = gdp->pointGroups();
                                    GB_Group *curr = pointGroups0.head();
                                    while( curr )
                                    {
                                        GB_Group *tmp = curr;
                                        curr = (GB_Group*)curr->next();
                                        
                                        if ( tmp->contains( pt0Next ) )
                                        {
                                            gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                                        }  // if
                                    }  // while
                                    
                                    pts.remove( pt0Next );
                                    numPts = pts.entries();
                                    
                                    pt0Next = pt0NextNext;
                                    nextPos0 = nextNextPos0;
                                    iNext0 = pts.find( pt0Next );
                                    iNextNext0 = iNext0;
                                }  // if
                                else
                                {
                                    break;
                                }
                            }  // for n
                            
                            // If the angle between vectors pt1->pt0Next and p1->pt1Next is less than 120 degrees, replace pt1 with pt1Next
                            int i1 = pts.find( pt1 );
                            int iNext1 = ( i1 - dir + numPts ) % numPts;
                            for ( int n = 0; n < numPts-2; n++, iNext1 = (iNext1-dir+numPts)%numPts )
                            {
                                GEO_Point* pt1Next = pts.entry( iNext1 );
                                UT_Vector4 nextPos1 = pt1Next->getPos();
                                UT_Vector4 v1N1 = nextPos1 - pos1;
                                UT_Vector4 v1N0 = nextPos0 - pos1;
                                if ( v1N1.dot( v1N0 ) > -0.5 ) //&& norm1.dot( v1N1 ) > 0 )
                                {
                                    // Get rid of pt1's groups
                                    GB_GroupList& pointGroups1 = gdp->pointGroups();
                                    GB_Group *curr = pointGroups1.head();
                                    while( curr )
                                    {
                                        GB_Group *tmp = curr;
                                        curr = (GB_Group*)curr->next();
                                        
                                        if ( tmp->contains( pt1 ) )
                                        {
                                            gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                                        }  // if
                                    }  // while
                                    
                                    pts.remove( pt1 );
                                    numPts = pts.entries();
                                    
                                    pt1 = pt1Next;
                                    pos1 = nextPos1;
                                }  // if
                                else
                                {
                                    break;
                                }
                            }  // for n
                            
                            /*// Delete points that are no longer on the wavefront
                            cout << "   deleting pts ";
                            int numPtsToDelete = ptsToDelete.entries();
                            for ( int dp = 0; dp < numPtsToDelete; dp++ )
                            {
                                cout << ptsToDelete[dp]->getNum() << " ";
                                pts.remove( ptsToDelete[dp] );
                            }*/
                            
                            // Set up new group with pt1 and pt0Next
                            //cout << "and adding " << pt0Next->getNum() << endl;
                            //cout << "      dir = " << dir << endl;
                            UT_String newGroupName( "group_" );
                            int newGroupNum = nextGroupNum++;          // A new group is being added, so the number of groups increments
                            char numstr[UT_NUMBUF];
                            UT_String::itoa( numstr, newGroupNum );
                            newGroupName += numstr;
                            GB_PointGroup* newGroup = gdp->newPointGroup( newGroupName );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                            //cout << "      created a group " << newGroupName << endl;
                            
                            // Get rid of pt0
                            //UT_Vector4 curPos = curPt->getPos();
                            //fpreal dist0N = ( curPos - pos0 ).length();
                            //fpreal dist1N = ( curPos - pos1 ).length();
                            
                            newGroup->add( pt1 );
                            newGroup->add( pt0Next );
                            
                        }  // if
                        else        // pt1 is closer to curPt than pt0, so get rid of pt1
                        {
                            //cout << "      deleting pt1 #" << pt1->getNum() << " ";
                            
                            int i1 = pts.find( pt1 );
                            int iNext1 = ( i1 - dir + numPts ) % numPts;
                            GEO_Point* pt1Next = pts.entry( iNext1 );
                            UT_Vector4 nextPos1 = pt1Next->getPos();
                            
                            // Find iNext, the next neighbor to pt1 that has a point number greater than pt1 (younger than pt1)
                            /*for ( int n = 0; n < numPts-2; n++, iNext = (iNext-dir+numPts)%numPts )
                            {
                                nextPt = pts.entry( iNext );
                                nextPos = nextPt->getPos();
                                UT_Vector4 tmpPos = computeChildPosition( pos0, nextPos, norm0, sphRad );
                                fpreal tmpDist = ( pos0 - tmpPos ).length();
                                if ( tmpDist >= 2 * sphRad )
                                    break;
                                
                                //nextPt = pts.entry( iNext );
                                //if ( pt1->getNum() < nextPt->getNum() )     // pt0 will not be replaced by an older point
                                //    break;
                                
                                // If the point to replace pt0 in the group (nextPt) is older than pt0, it cannot replace pt0 and must be deleted with all its groups
                                GB_GroupList& pointGroups0 = gdp->pointGroups();
                                GB_Group *curr = pointGroups0.head();
                                while( curr )
                                {
                                    GB_Group *tmp = curr;
                                    curr = (GB_Group*)curr->next();
                                    
                                    if ( tmp->contains( nextPt ) )
                                    {
                                        gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                                    }  // if
                                }  // while
                                
                                pts.remove( nextPt );
                                numPts = pts.entries();
                                
                            }  // for n
                            */
                            
                            // Get rid of pt1's groups
                            GB_GroupList& pointGroups1 = gdp->pointGroups();
                            GB_Group *curr = pointGroups1.head();
                            while( curr )
                            {
                                GB_Group *tmp = curr;
                                curr = (GB_Group*)curr->next();
                                
                                if ( tmp->contains( pt1 ) )
                                {
                                    gdp->destroyPointGroup( (GB_PointGroup*)tmp );    
                                }  // if
                            }  // while
                            
                            // Get rid of pt1
                            pts.remove( pt1 );
                            numPts = pts.entries();
                            
                            // If the angle between vectors pt1Next->pt0 and pt1Next->pt1NextNext is less than 120 degrees, replace pt1Next with pt1NextNext
                            iNext1 = pts.find( pt1Next );
                            int iNextNext1 = ( iNext1 - dir + numPts ) % numPts;
                            for ( int n = 0; n < numPts-2; n++, iNextNext1 = (iNextNext1-dir+numPts)%numPts )
                            {
                                GEO_Point* pt1NextNext = pts.entry( iNextNext1 );
                                UT_Vector4 nextNextPos1 = pt1NextNext->getPos();
                                UT_Vector4 vN10 = pos0 - nextPos1;
                                UT_Vector4 vN1NN1 = nextNextPos1 - nextPos1;
                                if ( vN10.dot( vN1NN1 ) > -0.5 ) // && norm1.dot( vN10 ) > 0 )
                                {
                                    // Get rid of pt0's groups
                                    GB_GroupList& pointGroups0 = gdp->pointGroups();
                                    GB_Group *curr = pointGroups0.head();
                                    while( curr )
                                    {
                                        GB_Group *tmp = curr;
                                        curr = (GB_Group*)curr->next();
                                        
                                        if ( tmp->contains( pt1Next ) )
                                        {
                                            gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                                        }  // if
                                    }  // while
                                    
                                    pts.remove( pt1Next );
                                    numPts = pts.entries();
                                    
                                    pt1Next = pt1NextNext;
                                    nextPos1 = nextNextPos1;
                                    iNext1 = pts.find( pt1Next );
                                    iNextNext1 = iNext1;
                                }  // if
                                else
                                {
                                    break;
                                }
                            }  // for n
                            
                            // If the angle between vectors p0->pt1Next and p0->pt0Next is less than 120 degrees, replace pt0 with pt0Next
                            int i0 = pts.find( pt0 );
                            int iNext0 = ( i0 + dir + numPts ) % numPts;
                            for ( int n = 0; n < numPts-2; n++, iNext0 = (iNext0+dir+numPts)%numPts )
                            {
                                GEO_Point* pt0Next = pts.entry( iNext0 );
                                UT_Vector4 nextPos0 = pt0Next->getPos();
                                UT_Vector4 v0N0 = nextPos0 - pos0;
                                UT_Vector4 v0N1 = nextPos1 - pos0;
                                if ( v0N0.dot( v0N1 ) > -0.5 ) //&& norm0.dot( v0N0 ) > 0 )
                                {
                                    // Get rid of pt1's groups
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
                                    
                                    pts.remove( pt0 );
                                    numPts = pts.entries();
                                    
                                    pt0 = pt0Next;
                                    pos0 = nextPos0;
                                }  // if
                                else
                                {
                                    break;
                                }
                            }  // for n
                            
                            // Set up new group with pt1 and nextPt
                            //cout << "and adding " << pt1Next->getNum() << endl;
                            UT_String newGroupName( "group_" );
                            int newGroupNum = nextGroupNum++;          // A new group is being added, so the number of groups increments
                            char numstr[UT_NUMBUF];
                            UT_String::itoa( numstr, newGroupNum );
                            newGroupName += numstr;
                            GB_PointGroup* newGroup = gdp->newPointGroup( newGroupName );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                            //cout << "      created a group " << newGroupName << endl;
                            
                            // Get rid of pt1
                            //UT_Vector4 curPos = curPt->getPos();
                            //fpreal dist0N = ( curPos - pos0 ).length();
                            //fpreal dist1N = ( curPos - pos1 ).length();
                            
                            newGroup->add( pt0 );
                            newGroup->add( pt1Next );
                            
                        }  // else
                        
                        doContinue = true;
                        break;
                    }  // if
                
                }  // for c
                
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
                
                
                // If p0's normal intersects the new sphere, delete p0 and set p0's neighbor to p0
                /*if ( intersectRaySphere( pt0->getPos(), norm0, newPt->getPos(), sphRad ) )
                {
                    int iNext = ( pts.find( pt0 ) + dir + numPts ) % numPts;
                    GEO_Point* nextPt = pts.entry( iNext );
                    
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
                    
                    pt0 = nextPt;
                }  // if
                
                // If p1's normal intersects the new sphere, delete p1 and set p1's neighbor to p1
                if ( intersectRaySphere( pt0->getPos(), norm0, newPt->getPos(), sphRad ) )
                {
                    int iNext = ( pts.find( pt1 ) - dir + numPts ) % numPts;
                    GEO_Point* nextPt = pts.entry( iNext );
                    
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
                    
                    pt1 = nextPt;
                }  // if*/
                
                
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
    }  // if
    
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    
    return error();
}
