#include "SOP_ImportDOPGroups.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
#include <DOP/DOP_Engine.h>
#include <DOP/DOP_Parent.h>
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
    PRM_Name("doppath", "DOP Path"),
    PRM_Name("groupmask", "Group Mask"),
};

static PRM_Default          defEighteen(18);
static PRM_Default          defEmptyString( 0, "" );

PRM_Template
SOP_ImportDOPGroups::myTemplateList[] = {
    //PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0, &PRMscaleRange),
    //PRM_Template(PRM_FLT_J,	1, &names[1], &defEighteen, 0, &PRMscaleRange),
    PRM_Template( PRM_STRING, 1, &names[0], &defEmptyString ),
    PRM_Template( PRM_STRING, 1, &names[1], &defEmptyString ),
    PRM_Template(),
};




void
newSopOperator( OP_OperatorTable *table )
{
     table->addOperator(
                    new OP_Operator("importdopgroups",
					"Import DOP Groups",
					SOP_ImportDOPGroups::myConstructor,
					SOP_ImportDOPGroups::myTemplateList,
					0,
					1,
					0)
                        );
}




OP_Node *
SOP_ImportDOPGroups::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_ImportDOPGroups(net, name, op);
}




SOP_ImportDOPGroups::SOP_ImportDOPGroups( OP_Network *net, const char *name, OP_Operator *op )
	: SOP_Node(net, name, op), myGroup(0)
{
}

SOP_ImportDOPGroups::~SOP_ImportDOPGroups() {}




bool SOP_ImportDOPGroups::intersectRaySphere( UT_Vector4 rayOrigin, UT_Vector4 ray, UT_Vector4 sphCenter, fpreal radius )
{
    ray.normalize();
    
    UT_Vector4 sphVector = sphCenter - rayOrigin;
    
    fpreal distFromSphCenterToRay = ( ray.dot(sphVector) * ray - sphVector ).length();
    
    if ( distFromSphCenterToRay < radius )
        return true;
    else
        return false;

}  // intersectRaySphere




OP_ERROR SOP_ImportDOPGroups::cookMySop( OP_Context &context )
{//cout << "IMPORT: very start" << endl;
    GEO_Point   *ppt;
    
    // Before we do anything, we must lock our inputs.  Before returning,
    //	we have to make sure that the inputs get unlocked.
    //if ( lockInputs(context) >= UT_ERROR_ABORT )
    //    return error();
    
    this->flags().setTimeDep(1);
    
    float t = context.myTime;
    
    UT_String dopPath = DOPPATH(t);
    UT_String groupMask = GROUPMASK(t);
    
    // Duplicate our incoming geometry with the hint that we only
    // altered points.  Thus if we our input was unchanged we can
    // easily roll back our changes by copying point values.
    //duplicatePointSource(0, context);
    //cout << "IMPORT: in here" << endl;
    if ( error() < UT_ERROR_ABORT )
    {
        gdp->clearAndDestroy();
        
        OBJ_Node *objNode = OPgetDirector()->findOBJNode( dopPath );
        if ( objNode != NULL )
        {
            OBJ_DopNet* dopNetwork = objNode->castToOBJDopNet();
            //DOP_Parent* dopParent = dopNetwork->castToDOPParent();
            const DOP_Engine* engine = &dopNetwork->getEngine();
            //const DOP_Engine& engine = dopNetwork->getEngine();
            
            // Get the SIM_Engine's groups, masked by groupMask
            SIM_ConstDataArray dopGroups;
            UT_String filterGroupName = groupMask;
            filterGroupName += "*";
            SIM_DataFilterByName groupFilter( filterGroupName );
            engine->filterConstRelationships( groupFilter, dopGroups );
            
            int numOldNeighborGroups = dopGroups.entries();
            //cout << "IMPORT: " << numOldNeighborGroups << " groups" << endl;
            for ( int n = 0; n < numOldNeighborGroups; n++ )
            {
                // Get the DOP group
                SIM_Relationship* curDOPGroup =(SIM_Relationship*)dopGroups[n];
                
                // Make sure the curDOPGroup is of type SIM_RelationshipGroup
                UT_String relType = curDOPGroup->getRelationshipTypeData()->getDataType();
                if ( relType != "SIM_RelationshipGroup" )
                    continue;
                
                UT_String curDOPGroupName = curDOPGroup->getName();
                //cout << "found group " << curDOPGroupName << endl;
                // Create the SOP point group
                GB_PointGroup* curSOPGroup = gdp->newPointGroup( curDOPGroupName );
                
                // For each object in the DOP group, add a point to the SOP point group
                int numGroupObjs = curDOPGroup->getGroupEntries();
                
                for ( int i = 0; i < numGroupObjs; i++ )
                {
                    // Get the position of the DOP object
                    SIM_Object* currObject = (SIM_Object*)curDOPGroup->getGroupObject( i );
                    UT_Vector3 objPos;
                    currObject->getPosition()->getPosition( objPos );
                    
                    // Create a SOP point at the DOP object's position
                    ppt = gdp->appendPoint();
                    ppt->setPos( objPos );
                    
                    // Add the point to the SOP point group
                    curSOPGroup->add( ppt );
                    
                }  // for o
            }  // for n
        }  // if
    }  // if  
    // Unlocking the inputs that were locked at the start of this method
    //unlockInputs();
    
    gdp->notifyCache(GU_CACHE_ALL);
    
    return error();
}
