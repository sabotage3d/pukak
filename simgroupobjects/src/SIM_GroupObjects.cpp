#include "SIM_GroupObjects.h"


#include <OP/OP_Node.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipGroup.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_XformOrder.h>





void initializeSIM(void *)
{
    //register our stuff with houdini
    IMPLEMENT_DATAFACTORY(SIM_GroupObjects);
}


SIM_GroupObjects::SIM_GroupObjects(const SIM_DataFactory *factory)
: BaseClass(factory), SIM_OptionsUser(this)
{
}


SIM_GroupObjects::~SIM_GroupObjects()
{
}


const SIM_DopDescription*
SIM_GroupObjects::getGroupObjectsDopDescription()
{
    static PRM_Name         theGroupName(SIM_NAME_GROUP_NAME, "Group Name");
    static PRM_Name         theDoCollidingObjects(SIM_NAME_DO_COLLIDING_OBJECTS, "Do Colliding Objects");
    
    static PRM_Default         defGroupName( 0, "bob" );
    static PRM_Default         defZero(0);
    
    static PRM_Template        theTemplates[] = {
        PRM_Template(PRM_STRING,        1, &theGroupName, &defGroupName ),
        PRM_Template(PRM_TOGGLE_J,      1, &theDoCollidingObjects, &defZero),
        PRM_Template()
    };
       
    static SIM_DopDescription   theDopDescription(true,
                                    "simgroupobjects",
                                    "Group Solver",
                                    "Group Solver",
                                    classname(),
                                    theTemplates);
       
    return &theDopDescription;
}  // getGroupObjectsDopDescription


SIM_Solver::SIM_Result SIM_GroupObjects::solveObjectsSubclass(SIM_Engine &engine,
                                                         SIM_ObjectArray &objects,
                                                         SIM_ObjectArray &newobjects,
                                                         SIM_ObjectArray &feedbacktoobjects,
                                                         const SIM_Time &currTime)
{
    // Get the name of the group we're creating/adding to
    OP_Node* thisSolverNode = getCreatorNode();
    UT_String groupName;
    thisSolverNode->evalString( groupName, SIM_NAME_GROUP_NAME, 0, currTime );
    bool doCollidingObjects = thisSolverNode->evalInt( SIM_NAME_DO_COLLIDING_OBJECTS, 0, currTime );
    
    // Set up the relationship object for adding group
    SIM_Relationship *relationship;
    
    // Loop through all the objects that passed the filter.
    for( int i = 0; i < objects.entries(); i++ )
    {
        bool addToGroup = false;
        
        if ( doCollidingObjects )
        {
            int numImpacts = 0;
            SIM_Impacts* impactsData = SIM_DATA_GET( *(objects(i)), "Impacts", SIM_Impacts );
            if ( impactsData )
            {
                numImpacts = impactsData->getNumImpacts();
            }  // if
            if ( numImpacts > 0 )
            {
                addToGroup = true;
            }  // if
        }  // if
        
        // Create the relationship if it doesn't already exist,
        // and add the object to the group.
        if( addToGroup )
        {
            relationship = engine.addRelationship(groupName,
                              SIM_DATA_RETURN_EXISTING);
            if ( relationship )
            {
                relationship->addGroup(objects(i));
                SIM_DATA_CREATE(*relationship, SIM_RELGROUP_DATANAME,
                                SIM_RelationshipGroup,
                                SIM_DATA_RETURN_EXISTING);
            }  // if
        }  // if
        
    }  // for i
    
    return SIM_Solver::SIM_SOLVER_SUCCESS;
}  // solveObjectsSubclass()




