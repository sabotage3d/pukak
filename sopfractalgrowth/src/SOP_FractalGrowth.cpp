#include "SOP_FractalGrowth.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
//#include <UT/UT_Matrix3.h>
//#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimSphere.h>
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
    //PRM_Name("phase",	"Phase"),
    //PRM_Name("period",	"Period"),
};

PRM_Template
SOP_FractalGrowth::myTemplateList[] = {
    PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0, &PRMscaleRange),
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




OP_ERROR SOP_FractalGrowth::cookMySop( OP_Context &context )
{
    GEO_Point   *ppt;
    
    int sphRad = RADIUS();
    
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
        for ( int i = 0; i < 40; i++ )
        {
            // Get a list of all the point groups
            GB_GroupList& pointGroups = gdp->pointGroups();
            int numGroups = pointGroups.length();
            cout << "numgroups = " << numGroups << endl;
            
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
            cout << endl;
            
            // If there are still point groups left, do a fractal growth;
            //   Otherwise, exit the loop
            if ( numGroups > 0 )
            {
                // Choose one random group out of the existing point groups
                int randomNumber = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
                int randGroupIndex = randomNumber % numGroups;
                GB_PointGroup* curGroup = (GB_PointGroup*)( ((UT_LinkList&)pointGroups).find(randGroupIndex) );
                
                cout << "   Cur group = " << " ";
                GEO_Point *bob;
                FOR_ALL_OPT_GROUP_POINTS( gdp, curGroup, bob )
                {
                    cout << bob->getNum() << "  ";
                }
                cout << endl;
                
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
                int norm_index = gdp->findPointAttrib( "N", 3 * sizeof(float), GB_ATTRIB_VECTOR );
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
                
                // Average the points' normals (to be assigned to the fractally grown particle)
                UT_Vector4 avgNorm( (norm0.x()+norm1.x())/2.0, (norm0.y()+norm1.y())/2.0, (norm0.z()+norm1.z())/2.0, 1 );
                newPt->castAttribData<UT_Vector3>( norm_index )->x() = avgNorm.x();  //(norm0.x()+norm1.x())/2.0;
                newPt->castAttribData<UT_Vector3>( norm_index )->y() = avgNorm.y();  //(norm0.y()+norm1.y())/2.0;
                newPt->castAttribData<UT_Vector3>( norm_index )->z() = avgNorm.z();  //(norm0.z()+norm1.z())/2.0;
                
                // Figure out which direction to go around the ring
                int dir = 0;
                //UT_Vector4 pt0pt1 = pt0->getPos() - pt1->getPos();
                //if ( cross( pt0pt1, avgNorm ).y() < 0 )
                //    dir = 1;
                //else
                //    dir = -1;
                if ( pts.find( pt0 ) > pts.find( pt1 ) )
                    dir = 1;
                else
                    dir = -1;
                
                // Set up data for computing which particles to group the new child particle with
                int numPts = pts.entries();
                //int minDist = 4 * sphRad * sphRad;    // (2r)^2 is the minimum squared distance that the new sphere's groups' particles can be from any other sphere in the wavefront
                int minDist = 2 * sphRad;               // (2r) is the minimum squared distance that the new sphere's groups' particles can be from any other sphere in the wavefront
                
                cout << "   pts = ";
                for ( int p = 0; p < numPts; p++ )
                    cout << pts[p]->getNum() << " ";
                cout << endl;
                
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
                        cout << "   Collided with " << curPt->getNum() << endl;
                        cout << "      " << dist << " < " << minDist << endl;
                        cout << "      destroying group" << endl;
                        gdp->destroyPointGroup( curGroup );
                        
                        UT_Vector4 pos0 = pt0->getPos();
                        UT_Vector4 pos1 = pt1->getPos();
                        
                        fpreal dist0 = ( curPos - pos0 ).length();
                        fpreal dist1 = ( curPos - pos1 ).length();
                        
                        int rand01 = rand();  // Returns a random int (from some C++ determined range, possibly -MAXINT to MAXINT)
                        rand01 = rand01 % 2;
                        if ( pt0->getNum() < pt1->getNum() )   // Use pt0
                        {
                            // Get rid of pt0 groups
                            cout << "      deleting pt0 #" << pt0->getNum() << " ";
                                
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
                            
                            // Set up new group with pt1 and nextPt
                            int i0 = pts.find( pt0 );
                            int iNext = ( i0 + dir + numPts ) % numPts;
                            GEO_Point* nextPt = pts.entry( iNext );
                            cout << "and adding " << nextPt->getNum() << endl;
                            cout << "      dir = " << dir << endl;
                            UT_String newGroupName( "group_" );
                            int newGroupNum = nextGroupNum++;          // A new group is being added, so the number of groups increments
                            char numstr[UT_NUMBUF];
                            UT_String::itoa( numstr, newGroupNum );
                            newGroupName += numstr;
                            GB_PointGroup* newGroup = gdp->newPointGroup( newGroupName );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                            cout << "      created a group " << newGroupName << endl;
                            newGroup->add( pt1 );
                            newGroup->add( nextPt );
                            
                            // Get rid of pt0
                            pts.remove( pt0 );
                        }
                        else        // Use pt1
                        {
                            // Get rid pt1 groups
                            cout << "      deleting pt1 #" << pt1->getNum() << " ";
                            
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
                            
                            // Set up new group with pt1 and nextPt
                            int i1 = pts.find( pt1 );
                            int iNext = ( i1 - dir + numPts ) % numPts;
                            GEO_Point* nextPt = pts.entry( iNext );
                            cout << "and adding " << nextPt->getNum() << endl;
                            UT_String newGroupName( "group_" );
                            int newGroupNum = nextGroupNum++;          // A new group is being added, so the number of groups increments
                            char numstr[UT_NUMBUF];
                            UT_String::itoa( numstr, newGroupNum );
                            newGroupName += numstr;
                            GB_PointGroup* newGroup = gdp->newPointGroup( newGroupName );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                            cout << "      created a group " << newGroupName << endl;
                            newGroup->add( pt0 );
                            newGroup->add( nextPt );
                            
                            // Get rid of pt1
                            pts.remove( pt1 );
                        }
                        
                        doContinue = true;
                        break;
                    }  // if
                
                }  // for c
                
                if ( doContinue )
                {
                    gdp->deletePoint( newPt->getNum() );
                    continue;
                }
                
                /*
                // Get rid of the group between the two parents that are creating the new child
                cout << "  destroying group ";
                GEO_Point *bob;
                FOR_ALL_OPT_GROUP_POINTS( gdp, curGroup, bob )
                {
                    cout << bob->getNum() << ",";
                }
                cout << endl;
                
                gdp->destroyPointGroup( curGroup );
                
                int numErrs = 0;
                
                GB_GroupList& pointGroups0 = gdp->pointGroups();
                // Find the first point in the wavefront to be paired with the child particle
                //   (It's the first point found going counterclockwise that is farther than a sphere's distance away from all other spheres on the wavefront)
                cout << "  pt0 num = " << pt0->getNum() << endl;
                int indexPt0 = (pts.find( pt0 ) + dir + numPts) % numPts;
                cout << "   ";
                GEO_PointList ptsToDelete;
                for ( int c = 0; c < numPts-2; c++, indexPt0 = (indexPt0+dir+numPts)%numPts )
                {
                    GEO_Point* curPt = pts.entry( indexPt0 );
                    cout << curPt->getNum() << " ";
                    UT_Vector4 curPos = curPt->getPos();
                    
                    UT_Vector4 pt0Pos = pt0->getPos();
                    UT_Vector4 pt1Pos = pt1->getPos();
                    
                    // C = curPt
                    // N = newPt
                    // 0 = pt0
                    // 1 = pt1
                    UT_Vector4 vC0 = pt0Pos - curPos;
                    UT_Vector4 vCN = newPos - curPos;
                    UT_Vector4 vN0 = pt0Pos - newPos;
                    UT_Vector4 v01 = pt1Pos - pt0Pos;
                    UT_Vector4 v0N = newPos - pt0Pos;
                    UT_Vector4 v0C = curPos - pt0Pos;
                    vC0.normalize();
                    vCN.normalize();
                    vN0.normalize();
                    v01.normalize();
                    v0N.normalize();
                    v0C.normalize();
                    fpreal dotProdC0_CN = vC0.dot( vCN );
                    fpreal dotProdC0_N0 = vC0.dot( vN0 );
                    fpreal dotProdC0_01 = vC0.dot( v01 );
                    fpreal dotProd0N_0C = v0N.dot( v0C );
                    
                    fpreal distPt0ToCur = ( pt0Pos - curPos ).length();    // Get squared length from pt0 and the curPt
                    fpreal distNewToCur = ( newPos - curPos ).length();  // Get squared length from the newPt to the curPt
                    
                    cout << " *dists0 " << distPt0ToCur << " " << distNewToCur << "* ";
                    if ( distNewToCur < minDist && dotProdC0_CN > -0.5 && dotProd0N_0C > -0.5 && dotProdC0_01 > -0.5 && dotProdC0_N0 > -0.333333 )
                    {
                        GEO_Point* oldPt = pt0;
                        pt0 = curPt;
                        
                        GB_Group *curr = pointGroups0.head();
                        while( curr )
                        {
                            GB_Group *tmp = curr;
                            curr = (GB_Group*)curr->next();
                            
                            if ( tmp->contains( oldPt ) )
                            {
                                cout << " *destroying for 0 " << " ";
                                GEO_Point *ppt;
                                GB_PointGroup* pg = (GB_PointGroup*)tmp;
                                FOR_ALL_OPT_GROUP_POINTS( gdp, pg, ppt )
                                {
                                    cout << ppt->getNum() << ",";
                                }
                                cout << "* ";
                                
                                gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                                
                            }  // if
                        }  // while
                        
                        if ( ptsToDelete.find( oldPt ) == -1 )
                            ptsToDelete.append( oldPt );
                        
                    }  // if
                }  // for
                cout << endl;
                
                GB_GroupList& pointGroups1 = gdp->pointGroups();
                // Find the second point in the wavefront to be paired with the child particle
                //   (It's the first point found going clockwise that is farther than a sphere's distance away from all other spheres on the wavefront)
                cout << "  pt1 num = " << pt1->getNum() << endl;
                int indexPt1 = (pts.find( pt1 ) - dir + numPts) % numPts;
                cout << "   ";
                for ( int c = 0; c < numPts-2; c++, indexPt1 = (indexPt1-dir+numPts)%numPts )
                {
                    GEO_Point* curPt = pts[indexPt1];
                    cout << curPt->getNum() << " ";
                    UT_Vector4 curPos = curPt->getPos();
                    
                    UT_Vector4 pt0Pos = pt0->getPos();
                    UT_Vector4 pt1Pos = pt1->getPos();
                    
                    // C = curPt
                    // N = newPt
                    // 0 = pt0
                    // 1 = pt1
                    UT_Vector4 vC1 = pt1Pos - curPos;
                    UT_Vector4 vCN = newPos - curPos;
                    UT_Vector4 vN1 = pt1Pos - newPos;
                    UT_Vector4 v10 = pt0Pos - pt1Pos;
                    UT_Vector4 v1N = newPos - pt1Pos;
                    UT_Vector4 v1C = curPos - pt1Pos;
                    vC1.normalize();
                    vCN.normalize();
                    vN1.normalize();
                    v10.normalize();
                    v1N.normalize();
                    v1C.normalize();
                    fpreal dotProdC1_CN = vC1.dot( vCN );
                    fpreal dotProdC1_N1 = vC1.dot( vN1 );
                    fpreal dotProdC1_10 = vC1.dot( v10 );
                    fpreal dotProd1N_1C = v1N.dot( v1C );
                    
                    fpreal distPt1ToCur = ( pt1Pos - curPos ).length();    // Get squared length from pt0 and the curPt
                    fpreal distNewToCur = ( newPos - curPos ).length();  // Get squared length from the newPt to the curPt
                    
                    cout << " *dists1 " << distPt1ToCur << " " << distNewToCur << "* ";
                    if ( distNewToCur < minDist && dotProdC1_CN > -0.5 && dotProd1N_1C > -0.5 && dotProdC1_10 > -0.5 && dotProdC1_N1 > -0.333333 )
                    {
                        GEO_Point* oldPt = pt1;
                        pt1 = curPt;
                        
                        GB_Group *curr = pointGroups1.head();
                        while( curr )
                        {
                            GB_Group *tmp = curr;
                            curr = (GB_Group*)curr->next();
                            if ( tmp->contains( oldPt ) )
                            {
                                cout << " *destroying for 1 " << " ";
                                GEO_Point *ppt;
                                GB_PointGroup* pg = (GB_PointGroup*)tmp;
                                FOR_ALL_OPT_GROUP_POINTS( gdp, pg, ppt )
                                {
                                    cout << ppt->getNum() << ",";
                                }
                                cout << "* ";
                                
                                gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                                
                            }  // if
                            
                        }  // while
                        
                        if ( ptsToDelete.find( oldPt ) == -1 )
                            ptsToDelete.append( oldPt );
                        
                    }  // if
                }  // for
                cout << endl;
                
                
                // Delete points that are no longer on the wavefront
                cout << "   deleting pts ";
                int numPtsToDelete = ptsToDelete.entries();
                for ( int dp = 0; dp < numPtsToDelete; dp++ )
                {
                    cout << ptsToDelete[dp]->getNum() << " ";
                    pts.remove( ptsToDelete[dp] );
                }
                cout << endl;
                */
                
                /*
                // Get rid of groups with pt0 and pt1 that have the new child particle within radius of its other point
                GB_Group *curr = pointGroups.head();
                while( curr )
                {
                    GB_Group *tmp = curr;
                    curr = (GB_Group*)curr->next();
                    
                    if ( tmp->contains( pt0 ) )
                    {
                        gdp->destroyPointGroup( (GB_PointGroup*)tmp );
                    }  // if
                }  // for
                */
                
                gdp->destroyPointGroup( curGroup );
                
                if ( pt0->getNum() == pt1->getNum() )
                    cout << "   YIKES: " << pt0->getNum() << " == " << pt1->getNum() << "!!!!!" << endl;
                
                // Set up new group with pt0
                UT_String newGroupName( "group_" );
                int newGroupNum = nextGroupNum++;          // A new group is being added, so the number of groups increments
                char numstr[UT_NUMBUF];
                UT_String::itoa( numstr, newGroupNum );
                newGroupName += numstr;
                GB_PointGroup* newGroup = gdp->newPointGroup( newGroupName );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                cout << "   created a group" << endl;
                newGroup->add( pt0 );
                newGroup->add( newPt );
                
                // Set up new group with pt1
                UT_String newGroupName1( "group_" );
                int newGroupNum1 = nextGroupNum++;          // A new group is being added, so the number of groups increments
                char numstr1[UT_NUMBUF];
                UT_String::itoa( numstr1, newGroupNum1 );
                newGroupName1 += numstr1;
                GB_PointGroup* newGroup1 = gdp->newPointGroup( newGroupName1 );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                cout << "   created a group" << endl;
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
                
                cout << "  inputting point " << newPt->getNum() << endl;
                
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