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
    //PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu),
    PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0,
				   &PRMscaleRange),
    //PRM_Template(PRM_FLT_J,	1, &names[1], PRMzeroDefaults),
    //PRM_Template(PRM_FLT_J,	1, &names[2], PRMoneDefaults),
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
        }
        else
        {
            rotY = UT_Matrix4( negcostheta,  0,    negsintheta, 0,
                               0,            1,    0,           0,
                               sintheta,     0,    negcostheta, 0,
                               0,            0,    0,           1  );
            
            p2p1Norm.rowVecMult( rotY );
            p2p1Norm *= hypotenuse;
            p2p1Norm = p1 + p2p1Norm;
        }
        
        //UT_Matrix4 rotZ( costheta, negsintheta, 0, 0,
        //                 sintheta, costheta,    0, 0,
        //                 0,        0,           1, 0,
        //                 0,        0,           0, 1  );
        
        //p2p1Norm.rowVecMult( rotX );
        //p2p1Norm.rowVecMult( rotY );
        //p2p1Norm.rowVecMult( rotZ );
        
        //p2p1Norm *= hypotenuse;
        
        //p2p1Norm = p2 + p2p1Norm;
    }
        
    return p2p1Norm;

}  // computeChildPosition()




OP_ERROR SOP_FractalGrowth::cookMySop( OP_Context &context )
{
    GEO_Point   *ppt;
    
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
        GB_GroupList& ptgrp = gdp->pointGroups();
        int nextGroupNum = ptgrp.length();
        
        for ( int i = 0; i < 18; i++ )
        {
            // Get all the geometry points
            GEO_PointList& pts = gdp->points();
            
            // Determine which point groups we have to work on.
            GB_GroupList& pointGroups = gdp->pointGroups();
            int numGroups = pointGroups.length();
            
            /*for ( int j = 0; j < numGroups; j++ )
            {
                cout << "    ";
                GEO_Point *ppt;
                GB_PointGroup* grp = (GB_PointGroup*)( ((UT_LinkList&)pointGroups).find(j) );
                FOR_ALL_OPT_GROUP_POINTS( gdp, grp, ppt )
                {
                    cout << ppt->getNum() << " ";
                }
                cout << endl;
            }*/
            
            if ( numGroups > 0 )
            {
                // Choose a random group out of the existing point groups
                int randomNumber = rand();
                int randGroupIndex = randomNumber % numGroups;
                GB_PointGroup* curGroup = (GB_PointGroup*)( ((UT_LinkList&)pointGroups).find(randGroupIndex) );
                
                // Array to keep track of the point positions in the current group
                UT_Vector4Array points;
                
                // Get the point positions of all the points in the selected group
                GEO_Point *ppt;
                GEO_Point *pt0;
                GEO_Point *pt1;
                fpreal numPts = 0;
                UT_Vector4 avgPos( 0.0, 0.0, 0.0 );
                FOR_ALL_OPT_GROUP_POINTS( gdp, curGroup, ppt )
                {
                    UT_Vector4 p = ppt->getPos();
                    points.append( p );
                    
                    if ( numPts == 0 )
                        pt0 = ppt;
                    else if ( numPts == 1 )
                        pt1 = ppt;
                    
                    avgPos += p;
                    numPts += 1;
                }
                
                // Average the points' positions
                //avgPos /= numPts;
                int norm_index = gdp->findPointAttrib( "N", 3 * sizeof(float), GB_ATTRIB_VECTOR );
                UT_Vector3* n0 = pt0->castAttribData<UT_Vector3>( norm_index );
                UT_Vector4 norm0( n0->x(), n0->y(), n0->z() );
                UT_Vector3* n1 = pt1->castAttribData<UT_Vector3>( norm_index );
                UT_Vector4 norm1( n1->x(), n1->y(), n1->z() );
                avgPos = computeChildPosition( points[0], points[1], norm0, 1 );
                
                // Now build a sphere between those points
                GEO_Point *newPt = gdp->appendPoint();
                newPt->getPos() = avgPos;
                
                GU_PrimSphereParms sphParms;
                sphParms.gdp = gdp;
                sphParms.ppt = newPt;
                GU_PrimSphere* newSphere = (GU_PrimSphere*)GU_PrimSphere::build( sphParms );
                //newPt = newSphere->getVertex(0).getPt();
                
                //UT_Vector4 newNorm( (norm0->x()+norm1->x())/2.0, (norm0->y()+norm1->y())/2.0, (norm0->z()+norm1->z())/2.0 );
                newPt->castAttribData<UT_Vector3>( norm_index )->x() = (norm0.x()+norm1.x())/2.0;
                newPt->castAttribData<UT_Vector3>( norm_index )->y() = (norm0.y()+norm1.y())/2.0;
                newPt->castAttribData<UT_Vector3>( norm_index )->z() = (norm0.z()+norm1.z())/2.0;
                
                // Create the new groups, the new point grouped with each point in the old group
                FOR_ALL_OPT_GROUP_POINTS( gdp, curGroup, ppt )
                {
                    UT_String newGroupName( "group_" );
                    int newGroupNum = nextGroupNum++;          // A new group is being added, so the number of groups increments
                    char numstr[UT_NUMBUF];
                    UT_String::itoa( numstr, newGroupNum );
                    newGroupName += numstr;
                    
                    GB_PointGroup* newGroup = gdp->newPointGroup( newGroupName );   //new GB_PointGroup( &pointGroups, newGroupName, 0 );    // 3rd parameter, hidden = 0
                    
                    if ( ppt->getNum() < newPt->getNum() )
                    {
                        newGroup->add( ppt );
                        newGroup->add( newPt );
                    }
                    else
                    {
                        newGroup->add( newPt );
                        newGroup->add( ppt );
                    }
                }
                
                //numGroups--;    // Decrement the number of groups, since we are going to delete curGroup
                gdp->destroyPointGroup( curGroup );
                
            }  // if
        }  // for
    }  // if
    
    // Unlocking the inputs that were locked at the start of this method
    unlockInputs();
    
    return error();
}