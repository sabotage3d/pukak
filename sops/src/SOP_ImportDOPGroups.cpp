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
    PRM_Name("groupmask", "Import Group Mask"),
	PRM_Name("interiorgranulesgrp", "Interior Granules Group"),
};

static PRM_Default          defEighteen(18);
static PRM_Default          defEmptyString( 0, "" );

PRM_Template
SOP_ImportDOPGroups::myTemplateList[] = {
    //PRM_Template(PRM_FLT_J,	1, &names[0], PRMoneDefaults, 0, &PRMscaleRange),
    //PRM_Template(PRM_FLT_J,	1, &names[1], &defEighteen, 0, &PRMscaleRange),
    PRM_Template( PRM_STRING, 1, &names[0], &defEmptyString, 0, 0, 0, 0, 1, "Path to the DOP node from which to import groups." ),
    PRM_Template( PRM_STRING, 1, &names[1], &defEmptyString, 0, 0, 0, 0, 1, "The mask of the DOP group(s) to import as SOP groups." ),
	PRM_Template( PRM_STRING, 1, &names[2], &defEmptyString, 0, 0, 0, 0, 1, "The name of the group which, if imported points are part of this group, those points are tagged as interior granules (optional)." ),
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
    UT_String importGroupMask = GROUPMASK(t);
	importGroupMask.strip("*");
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
            
            // Get the SIM_Engine's groups, masked by importGroupMask
            SIM_ConstDataArray dopGroups;
            UT_String filterGroupName = importGroupMask;
            filterGroupName += "*";
            SIM_DataFilterByName groupFilter( filterGroupName );
            engine->filterConstRelationships( groupFilter, dopGroups );
			
			// Get the interior group
			SIM_Relationship* interiorGranulesDOPGroup = (SIM_Relationship*)engine->getRelationship( interiorGranulesGroupName );
			int numInteriorGranules = 0;
			if ( interiorGranulesDOPGroup )
				numInteriorGranules = interiorGranulesDOPGroup->getGroupEntries();
            
            // Add point attributes to the gdp
            int negOne = -1;
			int zero = 0;
            //GB_AttributeRef objidAttrib = gdp->addPointAttrib( "objid", sizeof(int), GB_ATTRIB_INT, &negOne );
			//GB_AttributeRef isInteriorAttrib = gdp->addPointAttrib( "is_interior", sizeof(int), GB_ATTRIB_INT, &zero );
			GA_RWAttributeRef objidAttrib = gdp->addIntTuple( GA_ATTRIB_POINT, "objid", 1, GA_Defaults(-1) );		// tuple size = 1, default = -1
			GA_RWAttributeRef isInteriorAttrib = gdp->addIntTuple( GA_ATTRIB_POINT, "is_interior", 1, GA_Defaults(0) );		// tuple size = 1, default = -1
            
            int numOldNeighborGroups = dopGroups.entries();
            //cout << "IMPORT: " << numOldNeighborGroups << " groups" << endl;
            for ( int n = 0; n < numOldNeighborGroups; n++ )
            {
                // Get the DOP group
                SIM_Relationship* curDOPGroup =(SIM_Relationship*)dopGroups[n];
                
                // Create the SOP point group
                UT_String curDOPGroupName = curDOPGroup->getName();
                GA_PointGroup* curSOPGroup = gdp->newPointGroup( curDOPGroupName );
                
                // Make sure the curDOPGroup is of type SIM_RelationshipGroup
                UT_String relType = curDOPGroup->getRelationshipTypeData()->getDataType();
                if ( relType != "SIM_RelationshipGroup" )
                    continue;
                    
                //cout << "creating group " << curDOPGroup->getName() << endl;
                
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
                    
                    // Transfer the objid to the point
                    ppt->setValue<int>( objidAttrib, currObject->getObjectId() );
                    
                    // Add the point to the SOP point group
                    curSOPGroup->add( ppt );
					
					// If currObject is in the interior group, mark it as such
					if ( interiorGranulesDOPGroup )
					{
						UT_String grpName = curDOPGroupName;
						grpName.strip( importGroupMask );
						int interiorObjid = grpName.toInt();
						if ( currObject->getObjectId() == interiorObjid && interiorGranulesDOPGroup->getGroupHasObject( currObject ) )
						{
							ppt->setValue<int>( isInteriorAttrib, 1 );
						}  // if
					}  // if
                    
                }  // for o
            }  // for n
            
            if ( numOldNeighborGroups <= 0 )
            {
                gdp->newPointGroup( importGroupMask );
            }  // if
            
        }  // if
    }  // if  
    // Unlocking the inputs that were locked at the start of this method
    //unlockInputs();
    
    gdp->notifyCache(GU_CACHE_ALL);
    
    return error();
}

