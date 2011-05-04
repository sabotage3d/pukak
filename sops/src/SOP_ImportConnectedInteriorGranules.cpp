#include "SOP_ImportConnectedInteriorGranules.h"


#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
#include <DOP/DOP_Engine.h>
#include <DOP/DOP_Parent.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimSphere.h>
#include <GU/GU_PrimPoly.h>
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
    PRM_Name("groupmask", "Group Prefix"),
    PRM_Name("intgranulesgrp", "Interior Granules Group"),
};

static PRM_Default          defEighteen(18);
static PRM_Default          defEmptyString( 0, "" );

PRM_Template
SOP_ImportConnectedInteriorGranules::myTemplateList[] = {
    //PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0, &PRMscaleRange),
    //PRM_Template(PRM_FLT_J,	1, &names[1], &defEighteen, 0, &PRMscaleRange),
    PRM_Template( PRM_STRING, 1, &names[0], &defEmptyString ),
    PRM_Template( PRM_STRING, 1, &names[1], &defEmptyString ),
    PRM_Template( PRM_STRING, 1, &names[2], &defEmptyString ),
    PRM_Template(),
};




void
newSopOperator( OP_OperatorTable *table )
{
     table->addOperator(
                    new OP_Operator("importconnectedinteriorgranules",
					"Import Connected Interior DOPs",
					SOP_ImportConnectedInteriorGranules::myConstructor,
					SOP_ImportConnectedInteriorGranules::myTemplateList,
					0,
					1,
					0)
                        );
}




OP_Node *
SOP_ImportConnectedInteriorGranules::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_ImportConnectedInteriorGranules(net, name, op);
}




SOP_ImportConnectedInteriorGranules::SOP_ImportConnectedInteriorGranules( OP_Network *net, const char *name, OP_Operator *op )
	: SOP_Node(net, name, op), myGroup(0)
{
}

SOP_ImportConnectedInteriorGranules::~SOP_ImportConnectedInteriorGranules() {}




OP_ERROR SOP_ImportConnectedInteriorGranules::cookMySop( OP_Context &context )
{//cout << "IMPORT: very start" << endl;
    GEO_Point   *ppt;
    
    // Before we do anything, we must lock our inputs.  Before returning,
    //	we have to make sure that the inputs get unlocked.
    //if ( lockInputs(context) >= UT_ERROR_ABORT )
    //    return error();
    
    this->flags().setTimeDep(1);
    
    float t = context.myTime;
    //cout << "frame = " << t << endl;
    UT_String dopPath = DOPPATH(t);
    
    UT_String groupMask = GROUPMASK(t);
    UT_String groupPrefix = groupMask;
    groupPrefix.strip("*");
    //cout << "groupPrefix = " << groupPrefix << endl;
    int groupPrefixLength = groupPrefix.length();
    
    UT_String interiorGranulesGroupName = INTERIORGRANULESGROUP(t);
    
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
            UT_String filterGroupName = groupMask;		// groupMask is the group prefix name (e.g. neighborsOf*)
            filterGroupName += "*";
            SIM_DataFilterByName groupFilter( filterGroupName );
            engine->filterConstRelationships( groupFilter, dopGroups );
            
            // Add point attributes to the gdp
            int negOne = -1;
            int zero = 0;
            GB_AttributeRef objidAttrib = gdp->addPointAttrib( "objid", sizeof(int), GB_ATTRIB_INT, &negOne );
            GB_AttributeRef isInteriorAttrib = gdp->addPointAttrib( "is_interior", sizeof(int), GB_ATTRIB_INT, &zero );
            
            // Get the interior granules DOP group
            const SIM_Relationship* interiorGranulesDOPGroup = engine->getRelationship( interiorGranulesGroupName );
            if ( !interiorGranulesDOPGroup )
                return error();
            int numInteriorGroupObjs = interiorGranulesDOPGroup->getGroupEntries();
            
            int numOldNeighborGroups = dopGroups.entries();
            //cout << "IMPORT: " << numOldNeighborGroups << " groups" << endl;
            for ( int n = 0; n < numOldNeighborGroups; n++ )
            {
                GEO_Point   *intPpt = NULL;
                
                // Get the DOP group
                SIM_Relationship* curDOPGroup =(SIM_Relationship*)dopGroups[n];
                
                // Create the SOP point group
                UT_String curDOPGroupName = curDOPGroup->getName();
                GB_PointGroup* curSOPGroup = gdp->newPointGroup( curDOPGroupName );
                
                // Make sure the curDOPGroup is of type SIM_RelationshipGroup
                UT_String relType = curDOPGroup->getRelationshipTypeData()->getDataType();
                if ( relType != "SIM_RelationshipGroup" )
                    continue;
                    
                //cout << "creating group " << curDOPGroup->getName() << endl;
                
                // For each object in the DOP group, add a point to the SOP point group
                int numGroupObjs = curDOPGroup->getGroupEntries();
                
                // Get the interior granule in this group
                UT_String dopGroupName = curDOPGroup->getName();
                UT_String interiorGranuleObjidStr;
                //cout << "dopGroupName = " << dopGroupName << endl;
                dopGroupName.substr( interiorGranuleObjidStr, groupPrefixLength, 100 );     // Get rid of the prefix of the group name to get the number of the center granule
                //cout << "interiorGranuleObjidStr = " << interiorGranuleObjidStr << endl;
                int interiorGranuleObjid = interiorGranuleObjidStr.toInt();
                //cout << "interiorGranuleObjid = " << interiorGranuleObjid << endl;
                for ( int i = 0; i < numInteriorGroupObjs; i++ )
                {
                    SIM_Object* currObject = (SIM_Object*)interiorGranulesDOPGroup->getGroupObject( i );
                    if ( currObject->getObjectId() == interiorGranuleObjid )
                    {
                        UT_Vector3 objPos;
                        currObject->getPosition()->getPosition( objPos );
                        
                        // Create a SOP point at the DOP object's position
                        intPpt = gdp->appendPoint();
                        intPpt->setPos( objPos );
                        
                        // Transfer the objid to the point
                        intPpt->setValue<int>( objidAttrib, currObject->getObjectId() );
                        intPpt->setValue<int>( isInteriorAttrib, 1 );
                        
                        // Add the point to the SOP point group
                        curSOPGroup->add( intPpt );
                        
                        break;
                    }  // if
                }  // for i
                
                if ( !intPpt )
                {
                    //cout << "   intPpt is empty!!!!!" << endl;
                    continue;
                }  // if
                
                // Get all the neighbor granules in this group, and connect them to the interior granule with a prim if they are also an interior granule
                for ( int i = 0; i < numGroupObjs; i++ )
                {
                    SIM_Object* currObject = (SIM_Object*)curDOPGroup->getGroupObject( i );
                    if ( currObject->getObjectId() == interiorGranuleObjid )
                    {
                        continue;   // The center interior granule has already been added in the previous for loop
                    }  // if
                    
                    // Get the position of the DOP object
                    UT_Vector3 objPos;
                    currObject->getPosition()->getPosition( objPos );
                    
                    // Create a SOP point at the DOP object's position
                    ppt = gdp->appendPoint();
                    ppt->setPos( objPos );
                    
                    // Transfer the objid to the point
                    ppt->setValue<int>( objidAttrib, currObject->getObjectId() );
                    
                    // Add the point to the SOP point group
                    curSOPGroup->add( ppt );
                    //cout << "   adding " << currObject->getObjectId() << endl;
                    
                    // attach it to the center interior granule with a prim (for connectivity)
                    GU_PrimPoly *poly = (GU_PrimPoly*)gdp->appendPrimitive(GEOPRIMPOLY);
                    poly->appendVertex(intPpt);
                    poly->appendVertex(ppt);
                    
                    // If the current point is also an interior granule, 
                    if ( interiorGranulesDOPGroup->getGroupHasObject(currObject) )
                    {
                        ppt->setValue<int>( isInteriorAttrib, 1 );
                    }  // if
                    
                }  // for o
            }  // for n
            
            if ( numOldNeighborGroups <= 0 )
            {
                gdp->newPointGroup( groupMask );
            }  // if
            
        }  // if
    }  // if  
    // Unlocking the inputs that were locked at the start of this method
    //unlockInputs();
    
    gdp->notifyCache(GU_CACHE_ALL);
    
    return error();
}

