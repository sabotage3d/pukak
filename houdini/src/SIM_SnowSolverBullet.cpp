//
// Open Source Bullet Solver
//


#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h>
//#include <BulletCollision/CollisionShapes/shCompoundSphereShape.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#include <UT/UT_DSOVersion.h>
#include <UT/UT_XformOrder.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_SDF.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Options.h>
#include <SIM/SIM_PhysicalParms.h>
#include <SIM/SIM_ActiveValue.h>
#include <SIM/SIM_Data.h>
#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_Query.h>
#include <SIM/SIM_QueryArrays.h>                // Added by SRH 2010-05-29, for implementing multiple records in SIM_SnowNeighborData
#include <SIM/SIM_QueryCombine.h>               // Added by SRH 2010-05-29, for implementing multiple records in SIM_SnowNeighborData
#include <SIM/SIM_Utils.h>
#include <RBD/RBD_State.h>
#include <SIM/SIM_Container.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Force.h>
#include <SIM/SIM_ForcePoint.h>
#include <SIM/SIM_ForceUniform.h>
#include <SIM/SIM_ForceGravity.h>
#include <SIM/SIM_Constraint.h>
#include <SIM/SIM_ConAnchorObjSpacePos.h>
#include <SIM/SIM_ConAnchorWorldSpacePos.h>
#include <SIM/SIM_ConAnchor.h>
#include <SIM/SIM_ConRel.h>
#include <SIMZ/SIM_PopGeometry.h>
#include <GEO/GEO_Detail.h>
#include <GEO/GEO_PrimSphere.h>
#include <GB/GB_Group.h>
#include <OP/OP_Node.h>
#include <UT/UT_Quaternion.h>

#include "SIM_SnowSolverBullet.h"


#ifdef WIN32            // this prevents warnings when dependencies built
#include <windows.h>
#endif
    
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)


btTransform convertTobtTransform(const UT_DMatrix4 &inMat)
{
    UT_Matrix3 rotmatrix;
    inMat.extractRotate( rotmatrix );
    btMatrix3x3 btRotMatrix( rotmatrix(0,0), rotmatrix(0,1), rotmatrix(0,2),
                             rotmatrix(1,0), rotmatrix(1,1), rotmatrix(1,2),
                             rotmatrix(2,0), rotmatrix(2,1), rotmatrix(2,2) );
    UT_Vector3  translates;
    inMat.getTranslates( translates );
    btTransform btTrans;
    btTrans.setOrigin( btVector3( translates.x(),translates.y(),translates.z() ) );
    btTrans.setBasis( btRotMatrix );
    //return btTransform( btRotMatrix, btVector3( translates.x(),translates.y(),translates.z() ) );
    return btTrans;
}

//Constructor
SIM_SnowSolverBullet::SIM_SnowSolverBullet(const SIM_DataFactory *factory)
: BaseClass(factory), SIM_OptionsUser(this), state(0)
{
}

//Destructor
SIM_SnowSolverBullet::~SIM_SnowSolverBullet()
{
    if (state)
        state->removeReference();
}


//UI Param Def
const SIM_DopDescription *
SIM_SnowSolverBullet::getSolverBulletDopDescription()
{
    // ADDED BY SRH 2010-06-02 //
    static PRM_Name                theComputeImpacts(SIM_NAME_COMPUTE_IMPACTS, "Compute Impact Data");
    static PRM_Name                theSubsteps(SIM_NAME_SUBSTEPS, "Substeps for Bullet Solver");
       
    static PRM_Default             defOne(1);
       
    static PRM_Range               substepsRange(PRM_RANGE_UI, 1, PRM_RANGE_UI, 5);
       
    static PRM_Template            theTemplates[] = {
        PRM_Template(PRM_TOGGLE_J,  1, &theComputeImpacts, &defOne),
        PRM_Template(PRM_INT_J,     1, &theSubsteps, &defOne, 0, &substepsRange),
        PRM_Template()
    };
    // *********************** //
       
    //static PRM_Template   theTemplates[] = {
    //              PRM_Template()
    //};
    
    static SIM_DopDescription   theDopDescription(true,
        "bulletsnowsolver",
        "Bullet Snow Solver",
        "Solver",
        classname(),
        theTemplates);

    return &theDopDescription;
}



// -------------------------------------------------------------------
//      The solveObjectsSubclass is called once per simulation step.
//  The objects array has all of the objects you are to solve for.
SIM_Solver::SIM_Result SIM_SnowSolverBullet::solveObjectsSubclass(SIM_Engine &engine,
    SIM_ObjectArray &objects,
    SIM_ObjectArray &newobjects,
    SIM_ObjectArray &feedbacktoobjects,
    const SIM_Time &timestep)
{
    int i;
    UT_Vector3 p, v, w;
    UT_Quaternion rot;
    std::map< int, bulletBody >::iterator bodyIt;
    // ADDED BY SRH 2010-03-31 //
    std::map< int, bulletBody >::iterator affectorIt;
    // *********************** //
    btTransform btrans;
    int totalDopObjects = engine.getNumSimulationObjects();
    int totalUpdateObjects = engine.getNumSimulationObjects();
    float currTime = engine.getSimulationTime();
       
       
    // ADDED BY SRH 2010-03-30 FOR AFFECTOR CODE//
    SIM_ObjectArray collisionAffectors;
    const SIM_Geometry  *geometry = 0;
    // *********************** //


    if( !state )
    {
        //cerr<<"creating new state"<<endl;
        state = new SIM_SnowSolverBulletState();
        state->addReference();
    }
       
    // Remove any that have been deleted from dops in system: state->m_dynamicsWorld
    removeDeadBodies( engine );    
       
       
    // ADDED BY SRH 2010-04-01 - Now, instead of looping over all objects in the engine
    //                             it, only loops over objects directly connected to this Bullet Solver node. //
    //   First we add/update all objects for which we'll be solving using Bullet.
    //   (Objects using other solvers are not included here.)
    int numBulletObjects = objects.entries();   // "objects" was passed in as input to this function
    for ( i = 0; i < numBulletObjects; i++ )
    {
    // *********************** //
   
        //Get current dop sim object
        //SIM_Object *currObject = (SIM_Object *)engine.getSimulationObject(i);
        SIM_Object *currObject = (SIM_Object *)objects(i);
           
        //Check if this body has been added already, if not, add it.
        //Bullet bodies are stored in a map, the key is the dopObjectId
        bodyIt = state->m_bulletBodies->find( currObject->getObjectId() );
           
        //This is a new body (that is, it doesn't match any key in the map), make a bullet body to match
        if(bodyIt == state->m_bulletBodies->end())
            bodyIt = addBulletBody( currObject );
           
        if(bodyIt != state->m_bulletBodies->end())
        {
            // Pull the DOPs subdata and set the Bullet Transforms with it.        
            RBD_State *rbdstate = SIM_DATA_GET(*currObject, "Position", RBD_State);
            SIM_SnowBulletData *bulletstate = SIM_DATA_GET(*currObject, "Bullet Data", SIM_SnowBulletData); // UNCOMMENTED BY CHRIS 2010-06-08, FOR SLEEP THRESHOLD PARAMETERS
            
            // Pull initial Position & Rotation from subdata and set bullet transform
            p = rbdstate->getPosition();
            rot = rbdstate->getOrientation();
            btrans.setRotation( btQuaternion( rot.x(), rot.y(), rot.z(), rot.w() ) );
            btrans.setOrigin( btVector3(p[0],p[1],p[2]) );
            (bodyIt->second.bodyId)->getMotionState()->setWorldTransform(btrans);
            
            btScalar houMass = rbdstate->getMass();
            if( houMass > 0 )
            {
                //Velocity
                v = rbdstate->getVelocity();            
                (bodyIt->second.bodyId)->setLinearVelocity( btVector3(v[0],v[1],v[2]) ); 
                //Angular Velocity
                w = rbdstate->getAngularVelocity();
                (bodyIt->second.bodyId)->setAngularVelocity( btVector3(w[0],w[1],w[2]) );
                
                
                // ADDED BY SRH 2010-05-27 //
                //  If the mass of the Houdini RBD object has been turned from zero to nonzero,
                //  then update the Bullet objects mass.
                //Mass
                //cout << currObject->getName() << " mass = " << (bodyIt->second.bodyId)->getInvMass() << endl;
                if ( (bodyIt->second.bodyId)->getInvMass() == 0. )
                {
                    btVector3 fallInertia(0,0,0);
                    (bodyIt->second.bodyId)->getCollisionShape()->calculateLocalInertia( houMass, fallInertia );
                    (bodyIt->second.bodyId)->setMassProps( houMass, fallInertia );
                    
                    bodyIt->second.isStatic = false;
                }
                // *********************** //
            }
            
            
            // ADDED BY CHRIS 2010-06-07 *********** //
            // Add comment stuff //
            /*
            btScalar linearSleep = 0.1;
            btScalar angularSleep = 1.0f; 
            
            cout<<"Got to thresholds area."<<endl;
            
            cout<<bulletstate<<endl;
            OP_Node* thisbulletdataSolverNode = bulletstate->getCreatorNode();
            cout<<"Created thisbulletdataSolverNode"<<endl;
            //bool doChangeSleepThresholds = thisbulletdataSolverNode->evalInt( SIM_NAME_CHANGE_THRESHOLDS, 0, currTime );
            
            // Changes Sleeping Thresholds, OLD VERSION //
            
            cout<<bulletstate<<endl;
            OP_Node* thisbulletdataSolverNode = bulletstate->getCreatorNode();
            cout<<"Created thisbulletdataSolverNode"<<endl;
            bool doChangeSleepThresholds = thisbulletdataSolverNode->evalInt( SIM_NAME_CHANGE_THRESHOLDS, 0, currTime );                
                
            if (doChangeSleepThresholds)
            {
                cout<<"inside of sleepThresholds loop"<<endl;
                
                    linearSleep = thisbulletdataSolverNode->evalInt( SIM_NAME_LINEAR_SLEEP_THRESHOLD, 0, 1 );
                    
                    (bodyIt->second.bodyId)->setSleepingThresholds(linearSleep, angularSleep);
            }
            
            linearSleep = thisbulletdataSolverNode->evalInt( SIM_NAME_LINEAR_SLEEP_THRESHOLD, 0, 1 );
            (bodyIt->second.bodyId)->setSleepingThresholds(linearSleep, angularSleep);
             */     
            // ********************************** //
            
            
            // ADDED BY SRH43 2010-04-27 //
            //   If the mass is set to zero on any frame during the Houdini simulation,
            //   this should trigger the bullet object to freeze
                //   (it still affects other objects but is immovable and unaffected by others).
            //   To do this, the bullet object's mass is set to zero (essentially giving it infinite mass)
            //   and its velocities are set to zero.
            if ( (rbdstate->getMass() == 0 && !bodyIt->second.isStatic) )
            {
                (bodyIt->second.bodyId)->setMassProps( btScalar(0.0), btVector3(0.0, 0.0, 0.0) );
                (bodyIt->second.bodyId)->setLinearVelocity( btVector3(0.0, 0.0, 0.0) );
                (bodyIt->second.bodyId)->setAngularVelocity( btVector3(0.0, 0.0, 0.0) );
                (bodyIt->second.bodyId)->updateInertiaTensor();
                
                bodyIt->second.isStatic = true;
                // MAKE THE HOUDINI OBJECT INACTIVE ??????  NO!!!!!
                //(affectorIt->second.bodyId)->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            }
            // ************************* //
            
            //Add forces and such from the rest of the subdata
            processSubData(currObject, bodyIt->second);
        }
        
    }  // for each object(i)
    
    
    // ADDED BY SRH 2010-04-01 *************************************** //
    //   Based on the affector for-loop in the ODESolver code!  Thanks!
    //   IF AN AFFECTOR OBJECT IS NOT TAGGED AS "Is Kinematic" IN THE BULLET DATA NODE,
    //   ITS TRANSFORMS WILL NOT BE UPDATED IN BULLET
    for ( i = 0; i < numBulletObjects; i++ )
    {
        SIM_ColliderInfoArray colliders;
        SIM_Object *currAffector;
        //ObjectMap *odeaffectors = worlddata->getOdeAffectors();
        
        SIM_Object *currObject = (SIM_Object *)objects(i);
        
        currObject->getColliderInfo( colliders );
        bodyIt = state->m_bulletBodies->find( currObject->getObjectId() );
        for (int a = 0; a < colliders.entries(); a++)
        {
            currAffector = colliders(a).getAffector();
            int currAffectorId = currAffector->getObjectId();
            affectorIt = state->m_bulletAffectors->find( currAffectorId );
            if ( affectorIt == state->m_bulletAffectors->end() )    // The object is not already in the list of bullet affectors
            {
                bodyIt = state->m_bulletBodies->find( currAffectorId );
                
                if ( currAffector->getIsStatic()  )
                {
                    if ( bodyIt != state->m_bulletBodies->end() )
                    {
                        // This object has already been added as a Bullet body
                        // Make sure it is turned static in Bullet and collisions are still detected appropriately
                        (bodyIt->second.bodyId)->setMassProps( btScalar(0.0), btVector3(0.0, 0.0, 0.0) );
                        (bodyIt->second.bodyId)->setLinearVelocity( btVector3(0.0, 0.0, 0.0) );
                        (bodyIt->second.bodyId)->setAngularVelocity( btVector3(0.0, 0.0, 0.0) );
                        (bodyIt->second.bodyId)->updateInertiaTensor();
                        
                        bodyIt->second.isStatic = true;
                        //(affectorIt->second.bodyId)->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
                        
                        state->m_bulletAffectors->insert(std::make_pair( currAffectorId, bodyIt->second ));
                    }
                    else
                    {
                        // Add this affector object to the sim AS A BULLET BODY (not with addAffector(), but with addBulletBody())
                        //   (Since the Houdini is inactive, its mass will be set to zero and thus it will be inactive in Bullet)
                        //   Therefore, objects connected to this SIM_SnowSolverBullet node and objects
                        //   affecting those objects are treated equally, except the data (position, etc.)
                        //   is not updated for the affectors each frame, only when it is first added.
                        //   ***THIS WILL NEED TO BE FIXED IN THE FUTURE TO TAKE INTO AFFECT DYNAMIC AFFECTORS.***
                        affectorIt = addBulletBody( currAffector );
                        
                        (affectorIt->second.bodyId)->setMassProps( btScalar(0.0), btVector3(0.0, 0.0, 0.0) );
                        (affectorIt->second.bodyId)->setLinearVelocity( btVector3(0.0, 0.0, 0.0) );
                        (affectorIt->second.bodyId)->setAngularVelocity( btVector3(0.0, 0.0, 0.0) );
                        (affectorIt->second.bodyId)->updateInertiaTensor();
                        
                        affectorIt->second.isStatic = true;
                        //(affectorIt->second.bodyId)->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
                        
                        state->m_bulletAffectors->insert(std::make_pair( currAffectorId, affectorIt->second ));
                    }  // else
                } // if
            }  // if
        }  // for each affector(a) of the current object
    }  // for each object(i), look for additional affectors
    // *************************************************************** //
    
    
    
    // ADDED BY SRH 2010-06-02 //
    //   Get Bullet Data to 1) Determine if Impacts data should be computed
    //                      2) Determine the number of substeps for this simulation step to take
    //SIM_SnowBulletData *bulletstate = SIM_DATA_GET(*currObject, "Bullet Data", SIM_SnowBulletData);
    
    // ADDED BY SRH 2010-06-03 //
    // Get the parameter values from this Bullet Solver's node for computing impacts and getting the current substeps
    //   and set the matching data to those values.
    OP_Node* thisbulletSolverNode = getCreatorNode();
    bool doComputeImpacts = thisbulletSolverNode->evalInt( SIM_NAME_COMPUTE_IMPACTS, 0, currTime );
    int substeps = thisbulletSolverNode->evalInt( SIM_NAME_SUBSTEPS, 0, currTime );
    //setComputeImpacts( doComputeImpacts );
    //setSubsteps( substeps );
    // *********************** //
    
    //Run Sim and update DOPS not on init frame
    if(currTime > 0) 
    {
    
    
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // THIS IS WHERE IT ALL HAPPENS IN BULLET!!!!!! //
        // ALTERED BY SRH 2010-06-01 //
        //   Implement substepping by using the timestep of the simulation
        if ( substeps > 1 )
        {
            // If a Bullet Data node indicates higher than one substep, take that many substeps
            //   With 0 as the second arg in "stepSimulation()", Bullet does not take internal substeps.
            //   We do this because the accuracy of the simulation is faster this way.
            float stepSize = timestep / (float)substeps;
            for ( int i = 0; i < substeps; i++ )
            {
                state->m_dynamicsWorld->stepSimulation( stepSize, 0 );
            }
        }
        else
        {
            state->m_dynamicsWorld->stepSimulation( /*1/24.f*/ timestep, 10 );
        }
        // ************************* //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        // ******************************************** //
        
        
        
        // ADDED BY SRH 2010-04-29 //
        //   Attach each collision manifold to its corresponding Bullet rigid bodies.
        //   This is used later down the code to attach Impacts data to each rigid body.
        //   Manifolds are attached to their corresponding sim_btRigidBody objects by appending the manifold
        //     to the sim_btRigidBody->m_manifolds array variable
        //     (sim_btRigidBody class inherits from btRigidBody and is defined in SIM_SnowSolverBullet.h).
        //   m_manifolds is cleared out at the start of each step when removeDeadBodies() is called above.
        if ( doComputeImpacts )        // THIS IF-STATEMENT ADDED BY SRH 2010-06-03
        {
            btDispatcher* dispatcher = state->m_dynamicsWorld->getCollisionWorld()->getDispatcher();
            btPersistentManifold** manifoldPtr = dispatcher->getInternalManifoldPointer();
            int numManifolds = dispatcher->getNumManifolds();
            for ( int m = 0; m < numManifolds; m++ )
            {
                btPersistentManifold* manifold = manifoldPtr[m];
                
                // Extract the colliding objects from the manifold
                btCollisionObject* colObjA = (btCollisionObject*)manifold->getBody0();
                btCollisionObject* colObjB = (btCollisionObject*)manifold->getBody1();
                sim_btRigidBody* rbA = sim_btRigidBody::upcast(colObjA);
                sim_btRigidBody* rbB = sim_btRigidBody::upcast(colObjB);
                
                rbA->m_manifolds.push_back( manifold );
                rbB->m_manifolds.push_back( manifold );
                
            }  // for i
        }  // if
        // *********************** //
        
        
        
        //Iterate over active objects and update the dop geometry    
        //for    (i = 0;    i <    totalUpdateObjects; i++)
        for ( i = 0; i < numBulletObjects; i++ )
        {
            //Get current sim object
            //SIM_Object *currObject = (SIM_Object *)engine.getSimulationObject(i);
            // ADDED BY SRH 2010-04-01 //
            //   Now we only take active objects into consideration, instead of all objects
            SIM_Object *currObject = (SIM_Object *)objects(i);
            
            //Get State for Curr Object
            RBD_State    *rbdstate;
            rbdstate =  SIM_DATA_GET(*currObject, "Position", RBD_State);
            //SIM_SnowBulletData *bulletstate = SIM_DATA_GET(*currObject, "Bullet Data", SIM_SnowBulletData);
    
            //Get the matching bullet body in map
            bodyIt = state->m_bulletBodies->find( currObject->getObjectId() );
        
            if(bodyIt != state->m_bulletBodies->end())
            {    
                // Get the Bullet transform and update the DOPs subdata with it.
                (bodyIt->second.bodyId)->getMotionState()->getWorldTransform(btrans);
                // position
                UT_Vector3 p = UT_Vector3(
                    btrans.getOrigin().getX(), btrans.getOrigin().getY(), btrans.getOrigin().getZ() );
                rbdstate->setPosition( p );
                // rotation            
                UT_Quaternion rot = UT_Quaternion( btrans.getRotation().getX(), btrans.getRotation().getY(),
                    btrans.getRotation().getZ(), btrans.getRotation().getW() );
                rbdstate->setOrientation( rot );
                
                // Calculate mass and inertia
                btScalar mass = 1.0f;
                if( currObject->getIsStatic() )
                {
                    mass = 0;
                }
                else
                {
                    mass = rbdstate->getMass();
                }
                
                
                // ADDED BY SRH 2010-04-30 //
                //   Get Bullet impact data and add it to the Houdini object's data.
                //   (Add Impacts data for the current Houdini body, currObject,
                //   based on the current Bullet body's collision manifolds.)
                if ( doComputeImpacts )        // THIS IF-STATEMENT ADDED BY SRH 2010-06-03
                {
                    bool isImpact = false;
                    int numManifolds = (bodyIt->second.bodyId)->m_manifolds.size();
                    if ( numManifolds > 0 )
                    {
                        //SIM_SnowNeighborData* neighborData = SIM_DATA_CREATE( *currObject, "NeighborData", SIM_SnowNeighborData, SIM_DATA_RETURN_EXISTING );
                        SIM_SnowNeighborData* neighborData = SIM_DATA_GET( *currObject, "Bullet Neighbor Data", SIM_SnowNeighborData );
                        if ( !neighborData )
                        {
                            neighborData = SIM_DATA_CREATE( *currObject, "Bullet Neighbor Data", SIM_SnowNeighborData, 0 );
                        }
                        neighborData->resetNeighborIds();
                        
                        btAlignedObjectArray<int> neighborObjIDs;
                        UT_String neighborsStr = "[";        // This string keeps track of IDs of objects touching this object
                        //neighborsStr.sprintf( "%s", "[" );
                        
                        // ADDED BY SRH 2010-06-09 //
                        //  Set up a count for the number of static neighbors touching currObject
                        btAlignedObjectArray<int> staticNeighborObjIDs;
                        int numStaticNeighbors = 0;
                        // *********************** //
                        
                        // Set up the Impacts data field for currObject
                        //SIM_Impacts* impactsData = SIM_DATA_CREATE( *currObject, "Impacts", SIM_Impacts, SIM_DATA_RETURN_EXISTING );
                        //SIM_Impacts* impactsData = SIM_DATA_CREATE( *currObject, "Impacts", SIM_Impacts, 0 );
                        //impactsData->setNumImpacts( numManifolds );
                        for ( int m = 0; m < numManifolds; m++ )
                        {    
                            // For each collision (manifold) on the Bullet rigid body,
                            //    add the impact data to the Houdini rigid body Impacts field
                            btPersistentManifold* manifold = (bodyIt->second.bodyId)->m_manifolds[m];
                            
                            // NOTE: Just because a contact manifold exists does not mean there was an impact.
                            //         Sometimes, a manifold is created with zero contacts, so it does not contribute to Impacts data.
                            //         This is probably because in Bullet, each collision point that is diverging is removed from the manifold,
                            //         so zero contacts possibly means all contacts were diverging.
                            //         *OR* this is because the manifold represents another type of constraint, where there is not contact,
                            //         in which case this manifold is properly ignored.
                            //       So, Impacts data is added only if there are more than zero contacts in any of the manifolds.
                            
                            //btVector3 rBTot( 0, 0, 0 );
                            int numContacts = manifold->getNumContacts();
                            if ( numContacts > 0 )
                            {
                                isImpact = true;
                                
                                // For the current manifold, get the ID of the houdini object colliding with the current rigid body
                                sim_btRigidBody* rbA = (sim_btRigidBody*)manifold->getBody0();
                                sim_btRigidBody* rbB = (sim_btRigidBody*)manifold->getBody1();
                                int otherobjid = -1;
                                //bool otherobjisstatic = false;
                                if ( (bodyIt->second.bodyId) == rbA )
                                {
                                    otherobjid = rbB->houObjectId;
                                    //otherobjisstatic = ( rbB->mass == 0.f );
                                }
                                else
                                {
                                    otherobjid = rbA->houObjectId;
                                    //otherobjisstatic = ( rbA->mass == 0.f );
                                }
                                
                                // Add Impacts data for each impact on the current manifold
                                bool getDeepestImpactOnly = false;
                                if ( getDeepestImpactOnly )            // Get only the deepest contact point (first entry in the manifold's contacts array)
                                {
                                    SIM_Impacts* impactsData = SIM_DATA_CREATE( *currObject, "Impacts", SIM_Impacts, SIM_DATA_RETURN_EXISTING );
                                    btManifoldPoint& cp = manifold->getContactPoint( 0 );    // This gets the deepest contact point (contact points are ordered by depth, the deepest first)
                                    
                                    UT_Vector3 pos( cp.m_positionWorldOnB.x(), cp.m_positionWorldOnB.y(), cp.m_positionWorldOnB.z() );
                                    UT_Vector3 norm( cp.m_normalWorldOnB.x(), cp.m_normalWorldOnB.y(), cp.m_normalWorldOnB.z() );
                                    fpreal impulse = (fpreal)cp.m_appliedImpulse;
                                    SIM_Time cTime = (SIM_Time)currTime;
                                    
                                    impactsData->addPositionImpact( pos, norm, impulse, otherobjid, -1, -1, -1, -1, cTime, 0 );
                                    
                                    // Add Bullet Neighbor Data to keep track of all the objects the current object is touching on the current frame
                                    int numNeighbors = neighborObjIDs.size();
                                    bool foundMatch = false;
                                    for ( int i = 0; i < numNeighbors; i++ )
                                    {
                                        if ( neighborObjIDs[i] == otherobjid )
                                        {
                                            foundMatch = true;
                                            break;
                                        }
                                        
                                    } // for i
                                    
                                    if ( !foundMatch )
                                    {
                                        char idStr[50];
                                        int strSize = sprintf( idStr, "%d", otherobjid );
                                        if (strSize >= 50)
                                        {
                                            cout << "ERROR: string size is greater than the string buffer." << endl;
                                            return SIM_Solver::SIM_SOLVER_FAIL;
                                        }
                                        //neighborsStr.sprintf( "%s%d, ", neighborsStr.buffer(), otherobjid );
                                        neighborsStr += idStr;
                                        neighborsStr += ", ";
                                        
                                        neighborObjIDs.push_back( otherobjid );
                                        neighborData->appendNeighborId( otherobjid );
                                        
                                        // ADDED BY SRH 2010-06-09 //
                                        //   Keep count the number of static neighbors.
                                        std::map< int, bulletBody >::iterator neighborIt = state->m_bulletBodies->find( otherobjid );
                                        if ( neighborIt->second.isStatic )
                                        {
                                            staticNeighborObjIDs.push_back( otherobjid );
                                            numStaticNeighbors++;
                                        }  // if
                                        // *********************** //
                                    }
                                }
                                else    // Get all contact points
                                {
                                    SIM_Impacts* impactsData = SIM_DATA_CREATE( *currObject, "Impacts", SIM_Impacts, SIM_DATA_RETURN_EXISTING );
                                    for ( int n = 0; n < numContacts; n++ )        // For each contact point in the manifold
                                    {
                                        btManifoldPoint& cp = manifold->getContactPoint( n );
                                        
                                        UT_Vector3 pos( cp.m_positionWorldOnB.x(), cp.m_positionWorldOnB.y(), cp.m_positionWorldOnB.z() );
                                        UT_Vector3 norm( cp.m_normalWorldOnB.x(), cp.m_normalWorldOnB.y(), cp.m_normalWorldOnB.z() );
                                        fpreal impulse = (fpreal)cp.m_appliedImpulse;
                                        SIM_Time cTime = (SIM_Time)currTime;
                                        
                                        impactsData->addPositionImpact( pos, norm, impulse, otherobjid, -1, -1, -1, -1, cTime, 0 );
                                        
                                        //btVector3 rB = posB - colObjB->getWorldTransform().getOrigin();
                                        //printf( "rB %d = %f %f %f\n", j, rB.x(), rB.y(), rB.z() );
                                        //rBTot = btVector3( rBTot.x() + rB.x(), rBTot.y() + rB.y(), rBTot.z() + rB.z() );
                                        
                                        // Add Bullet Neighbor Data to keep track of all the objects the current object is touching on the current frame
                                    	int numNeighbors = neighborObjIDs.size();
                                    	bool foundMatch = false;
                                    	for ( int i = 0; i < numNeighbors; i++ )
                                   	 	{
                                     	   if ( neighborObjIDs[i] == otherobjid )
                                     	   {
                                    	        foundMatch = true;
                                    	        break;
                                    	    }
                                        
                                    	} // for i
                                    	
                                    	if ( !foundMatch )
                                    	{
                                    	    char idStr[50];
                                    	    int strSize = sprintf( idStr, "%d", otherobjid );
                                    	    if (strSize >= 50)
                                    	    {
                                    	        cout << "ERROR: string size is greater than the string buffer." << endl;
                                    	        return SIM_Solver::SIM_SOLVER_FAIL;
                                    	    }
                                    	    //neighborsStr.sprintf( "%s%d, ", neighborsStr.buffer(), otherobjid );
                                    	    neighborsStr += idStr;
                                    	    neighborsStr += ", ";
                                    	    
                                    	    neighborObjIDs.push_back( otherobjid );
                                    	    neighborData->appendNeighborId( otherobjid );
                                    	    
                                    	    // ADDED BY SRH 2010-06-09 //
                                    	    //   Keep count the number of static neighbors.
                                    	    std::map< int, bulletBody >::iterator neighborIt = state->m_bulletBodies->find( otherobjid );
                                    	    if ( neighborIt->second.isStatic )
                                    	    {
                                    	        staticNeighborObjIDs.push_back( otherobjid );
                                    	        numStaticNeighbors++;
                                    	    }  // if
                                    	    // *********************** //
                                    	}
                            
                                    }  // for n
                                }  // else
                                
                            }  // if
                        }  // for m
                        
                        // Assign neighbors data (indicates what objects are touching/colliding with the current object)
                        neighborsStr.strip( " " );
                        neighborsStr.replaceSuffix( ",", "" );
                        neighborsStr += "]";
                        //SIM_SnowNeighborData* neighborData = SIM_DATA_CREATE( *currObject, "NeighborData", SIM_SnowNeighborData, SIM_DATA_RETURN_EXISTING );
                        
                        int numNeighbors = neighborObjIDs.size();
                        bool foundMatch = false;
                        for ( int i = 0; i < numNeighbors; i++ )
                        {
                            char idStr[50];
                            int strSize = sprintf( idStr, "%d", neighborObjIDs[i]);
                            if (strSize >= 50)
                            {
                                cout << "ERROR: string size is greater than the string buffer." << endl;
                                return SIM_Solver::SIM_SOLVER_FAIL;
                            }
                            //neighborData->setGeoNeighbors( idStr );
                            
                        }  // for i
                        neighborData->setGeoNeighbors( (const UT_String)neighborsStr );
                        neighborData->setNumNeighbors( numNeighbors );
                        
                        /*// ADDED BY SRH 2010-06-09 //
                        //   If more than "max_static_neighbors" number of static neighbors were found,
                        //   this object is going to be deleted,
                        //   which means its neighbors will have one less neighbor,
                        //   so decrease the number of each neighbor's maximum neighbors allowed by one.
                        int maxStaticNeighbors = neighborData->getMaxStaticNeighbors();
                        if ( maxStaticNeighbors > 0 && numStaticNeighbors > maxStaticNeighbors )
                        {
                            // Mark currObject as needing to be deleted
                            neighborData->setDeleteMe( true );
                            
                            // Decrement each of currObject's static neighbors' maxStaticNeighbors by one
                            for ( int n = 0; n < numStaticNeighbors; n++ )
                            {
                                int nID = staticNeighborObjIDs[n];
                                SIM_Object* neighborObject = (SIM_Object*)engine.getSimulationObjectFromId( nID );
                                SIM_SnowNeighborData* neighborsNeighborData = SIM_DATA_CREATE( *neighborObject, "Bullet Neighbor Data", SIM_SnowNeighborData, SIM_DATA_RETURN_EXISTING );
                                neighborsNeighborData->setMaxStaticNeighbors( maxStaticNeighbors - 1 );
                            }  // for n
                        }  // if
                        // *********************** //*/
                        
                    }  // if
                    
                    if ( !isImpact )
                    {
                        SIM_Impacts* impactsData = SIM_DATA_GET( *currObject, "Impacts", SIM_Impacts );
                        if ( impactsData )
                        {
                            currObject->removeNamedSubData( "Impacts" );
                        }
                    }  // else
                    
                }  // if doComputeImpacts
                // ************************* //
                
                
                // CHANGED BY SRH 2010-05-25 //
                //  Since static objects are no longer in the list of updated objects,
                //  the "if" statement checking for static object is unneeded.
                //if( !(bodyIt->second.bodyId)->isStaticObject() && (bodyIt->second.bodyId)->isActive() == 1 ) // if not tagged as Static
                //{
                
                //Velocity
                btVector3 v;
                v = (bodyIt->second.bodyId)->getLinearVelocity(); 
                rbdstate->setVelocity( UT_Vector3( v.getX(), v.getY(), v.getZ() ) );
                
                //Angular Velocity
                btVector3 w;
                w = (bodyIt->second.bodyId)->getAngularVelocity(); 
                rbdstate->setAngularVelocity( UT_Vector3( w.getX(), w.getY(), w.getZ() ) );
                
                //}
                // ************************ //
                
                
            }    
        }
    }

                


    return SIM_Solver::SIM_SOLVER_SUCCESS;
}




//Adds a Bullet body to the world
//Based on type of geometry in the SIM_Object we make a Bullet body to match
std::map< int, bulletBody >::iterator SIM_SnowSolverBullet::addBulletBody(SIM_Object *currObject)
{
    std::map< int, bulletBody >::iterator bodyIt;
    bodyIt = state->m_bulletBodies->end();
    SIM_Geometry *myGeo;
    const SIM_SDF *sdf;
    UT_Vector3 p, v, w;
    UT_Quaternion rot;

    //cout<<"adding simulation object to bullet :"<<currObject->getName()<<endl;

    // Get the Geometry...  
    myGeo = (SIM_Geometry *)currObject->getGeometry();
       
    //We have geomoetry
    if(myGeo)
    {
        //Find out the type
        sdf = SIM_DATA_GETCONST(*myGeo, SIM_SDF_DATANAME, SIM_SDF);
        bulletBody      currBody;
           
        RBD_State *rbdstate = SIM_DATA_GET(*currObject, "Position", RBD_State);
        SIM_SnowBulletData *bulletstate = SIM_DATA_GET(*currObject, "Bullet Snow Data", SIM_SnowBulletData);
        if(rbdstate) //This is an rbd object
        {
            UT_DMatrix4 xform;
            SIMgetGeometryTransform(xform, *currObject);
            // TODO: support this xform. Currently I don't know how to get the CompoundShape, COM and such to work.
               
            // Retrieve the gdp...
            GU_DetailHandleAutoReadLock     gdl(myGeo->getGeometry());
            GU_Detail *gdp = (GU_Detail *)gdl.getGdp();    
            btCollisionShape* fallShape;
            fallShape = NULL;
            UT_BoundingBox bbox;
                           
            p = rbdstate->getPosition();
            rot = rbdstate->getOrientation();

            // default motion state, will be set by sim loop later
            btDefaultMotionState* fallMotionState = new btDefaultMotionState(
                    btTransform( btQuaternion(rot.x(),rot.y(),rot.z(),rot.w()),btVector3(p[0],p[1],p[2]) ) );
               
            UT_String geoRep;
            geoRep = GEO_REP_AS_IS;
            if( bulletstate )
            {
                bulletstate->getGeoRep(geoRep);
            }
               
            if( geoRep == GEO_REP_AS_IS ) // geometry as-is
            {
                GEO_Primitive *prim;
                GEO_Point *pt;
                GEO_PrimSphere *sphere;
                int ntriangles=0, nspheres=0;
                FOR_ALL_PRIMITIVES (gdp, prim)
                {
                    if (prim->getPrimitiveId() & GEOPRIMPOLY)
                        if( prim->getVertexCount() == 3 )  //add only tri's
                            ntriangles++;
                    if (prim->getPrimitiveId() & GEOPRIMSPHERE)
                        nspheres++;
                }
                if( ntriangles )
                {
                    //cout<<"creating triangle collision shape"<<endl;
                    // Transfer point positions
                    btVector3 *gVertices = new btVector3[gdp->points().entries()];
                    int numVertices = 0;
                    int vertStride = sizeof(btVector3);
                    FOR_ALL_GPOINTS( gdp, pt )
                    {
                        UT_Vector3 ppt = pt->getPos(); // * xform;
                        gVertices[numVertices++] = btVector3(
                            ppt.x(), ppt.y(), ppt.z() );
                    }
                    // Transfer Indices
                    int *gIndices = new int[gdp->primitives().entries()*3];
                    int indexStride = 3*sizeof(int);
                    int numIndices = 0;
                    FOR_ALL_PRIMITIVES (gdp, prim)
                        for( int i=0; i<3; i++ )
                            gIndices[numIndices++] = prim->getVertex(2-i).getPt()->getNum();
       
                    btTriangleIndexVertexArray* indexVertexArray = new btTriangleIndexVertexArray(
                        ntriangles,
                        gIndices,       indexStride,
                        numVertices, (btScalar*) &gVertices[0].x(), vertStride);

                    bool doConvex=false;
                    if( bulletstate )
                        doConvex = bulletstate->getGeoConvex();
                           
                    if( doConvex )
                    {
                        //convex-hull collision shape
                        fallShape = new btConvexTriangleMeshShape(indexVertexArray);
                    }
                    else
                    {                      
                        // concave collision shape
                        btGImpactMeshShape *impactShape = new btGImpactMeshShape(indexVertexArray);
                        impactShape->updateBound();
                        fallShape = impactShape;
                        btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>
                                (state->m_dynamicsWorld ->getDispatcher());
                        btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
                    }
                }
                else if ( nspheres == 1 )
                {
                    //cout<<"creating single sphere shape"<<endl;
                       
                    // FIXED BY SRH 2010-03-30 //
                    FOR_ALL_PRIMITIVES (gdp, prim)
                    {
                        sphere = dynamic_cast<GEO_PrimSphere*>( prim );
                           
                        // Set the sphere's scale (based off of both its radius setting and any scale transforms)
                        UT_Matrix4 xform;
                        sphere->getTransform4( xform );
                        UT_XformOrder xformOrder;
                        UT_Vector3 rotation, scale, translation;
                        xform.explode( xformOrder, rotation, scale, translation );
                        float sphereRadius = scale.x();
                        fallShape = new btSphereShape( sphereRadius );
                    }
                    // *********************** //
                }
                else if ( nspheres > 1 )
                {
                    // MAKE MULTIPLE SPHERES AND CONSTRAIN THEM
                    //   Uses srh's shCompoundSphereShape added to Bullet
                    //cout<<"creating multisphere shape"<<endl;
                   
                    // ADDED BY SRH 2010-02-22 //
                    btAlignedObjectArray<btSphereShape*> sphereShapes;
                    btAlignedObjectArray<btVector3> sphereRelativePositions;
                    fallShape = new btCompoundShape();
                    // *********************** //
                       
                    btScalar *sphereRadii = new btScalar [nspheres];
                    btVector3 *sphereCentres = new btVector3 [nspheres];
                    //btVector3 centerOfMass( 0.0, 0.0, 0.0 );
                    int c = 0;
                    UT_Vector3 centre;
                    FOR_ALL_PRIMITIVES (gdp, prim)
                    {
                        if (prim->getPrimitiveId() & GEOPRIMSPHERE)
                        {
                            sphere = dynamic_cast<GEO_PrimSphere*>( prim );
                               
                            // Sphere radius (based off of both its radius setting and any scale transforms)
                            // ADDED BY SRH 2010-02-22 //
                            UT_Matrix4 xform;
                            sphere->getTransform4( xform );
                            UT_XformOrder xformOrder;
                            UT_Vector3 rotation, scale, translation;
                            xform.explode( xformOrder, rotation, scale, translation );
                            sphereRadii[c] = scale.x();         // THIS NEEDS FIXED TO TAKE THE LARGEST (OR SMALLEST) DIMENSION (not just x)
                            // *********************** //
                               
                            // Sphere center
                            centre = sphere->baryCenter();
                            sphereCentres[c] = btVector3( centre.x(), centre.y(), centre.z() );
                            
                            // ADDED BY SRH 2010-02-22 //   
                            btSphereShape* sphShape = new btSphereShape( sphereRadii[c] );
                            btTransform curTransform;
                            curTransform.setIdentity();
                            curTransform.setOrigin( sphereCentres[c] );
                               
                            ((btCompoundShape*)fallShape)->addChildShape( curTransform, sphShape );
                            // *********************** //
                               
                            c++;
                        }
                           
                    }  // FOR_ALL_PRIMITIVES
                    
                    delete sphereRadii;
                    delete sphereCentres;
                }
                else if( ntriangles == 0  && nspheres == 0 )
                {
                    cout << "Bullet Solver: Warning! There are no spheres nor triangles in the simulation object:" << currObject->getName() << endl;
                }
            }
            else if( geoRep == GEO_REP_SPHERE )  // sphere representation
            {
                gdp->getBBox(&bbox);
                if( bulletstate->getAutofit() )
                    fallShape = new btSphereShape( bbox.maxvec().length() );
                else    
                    fallShape = new btSphereShape( fabs(bulletstate->getPrimRadius()) );
            }
            else if( geoRep == GEO_REP_BOX )  // box representation
            {
                int nspheres = 0;
                GEO_Primitive *prim;
                FOR_ALL_PRIMITIVES (gdp, prim)
                {
                    if (prim->getPrimitiveId() & GEOPRIMSPHERE)
                        nspheres++;
                }
                
                if ( nspheres == 0 )
                {
                    gdp->getBBox( &bbox );
                    if( bulletstate->getAutofit() )
                    {
                        // ADDED BY SRH 2010-06-28 //
                        // Retrieve the bounding box's scale information from Houdini to give the box appropriate scale
                        UT_DMatrix4 xform;
                        myGeo->getTransform( xform );
                        UT_XformOrder xformOrder;
                        UT_Vector3 rotation, scale, translation;
                        xform.explode( xformOrder, rotation, scale, translation );      // Get the object's scale
                        
                        // Compute the size and scale of the bounding box in all dimensions
                        //   bbox.size is divided by 2 to compute the radius of the bbox
                        float xScale = ( bbox.xsize() / 2.0 ) * scale.x();
                        float yScale = ( bbox.ysize() / 2.0 ) * scale.y();
                        float zScale = ( bbox.zsize() / 2.0 ) * scale.z();
                        
                        // This line was update by SRH to take the bounding box's scale into account
                        fallShape = new btBoxShape( btVector3( xScale, yScale, zScale ) );
                        // ************************ //
                    }
                    else
                    {
                        UT_Vector3 prim_s = bulletstate->getPrimS();
                        // UPDATED BY SRH 2010-06-28 //
                        //   It now divides the scale by two since the btBoxShape takes in the radii of the bounding box, not the diameter
                        fallShape = new btBoxShape( btVector3(prim_s.x()/2.0, prim_s.y()/2.0, prim_s.z()/2.0) );
                        // ************************* //
                    }
                }
                // ADDED BY SRH 2010-06-29 //
                //   If there are spheres, make a btCompound objects with a box at each Sphere
                //   (For now, this actually puts a box at each prim, even if they are not sphere prims).
                else
                {
                    fallShape = new btCompoundShape();
                    
                    //GB_PrimitiveGroup *grp = gdp->findPrimitiveGroup("group");
                    //FOR_ALL_GROUP_PRIMITIVES( gdp, grp, prim )
                    FOR_ALL_PRIMITIVES (gdp, prim)
                    {
                        gdp->getBBox( &bbox );
                        
                        int rot_index = gdp->findPrimAttrib( "rot", 3 * sizeof(float), GB_ATTRIB_VECTOR );
                        UT_Vector3* rot = prim->castAttribData<UT_Vector3>( rot_index );
                        
                        btMatrix3x3 rotMatrix;
                        if ( rot_index >= 0 )
                            rotMatrix = btMatrix3x3( btQuaternion(rot->x(), rot->y(), rot->z(), 1) );
                        else
                            rotMatrix.setIdentity();
                            
                        btTransform curTransform( rotMatrix );
                        
                        UT_Vector3 center = bbox.center();
                        curTransform.setOrigin( btVector3( center.x(), center.y(), center.z() ) );
                        
                        btBoxShape* boxShape;
                        if( bulletstate->getAutofit() )
                        {
                            UT_DMatrix4 xform;
                            myGeo->getTransform( xform );
                            UT_XformOrder xformOrder;
                            UT_Vector3 rotation, scale, translation;
                            xform.explode( xformOrder, rotation, scale, translation );      // Get the object's scale
                            
                            // Compute the size and scale of the bounding box in all dimensions
                            //   bbox.size is divided by 2 to compute the radius of the bbox
                            float xScale = ( bbox.xsize() / 4.0 ) * scale.x();
                            float yScale = ( bbox.ysize() / 4.0 ) * scale.y();
                            float zScale = ( bbox.zsize() / 4.0 ) * scale.z();
                            
                            //cout << "x y z scale = " << xScale << " " << yScale << " " << zScale << endl;
                            
                            boxShape = new btBoxShape( btVector3( xScale, yScale, zScale ) );
                            ((btCompoundShape*)fallShape)->addChildShape( curTransform, boxShape );
                        }
                        else
                        {
                            UT_Vector3 prim_s = bulletstate->getPrimS();
                            boxShape = new btBoxShape( btVector3(prim_s.x()/2.0, prim_s.y()/2.0, prim_s.z()/2.0) );
                        }
                        
                        ((btCompoundShape*)fallShape)->addChildShape( curTransform, boxShape );
                        
                    }
                }
                // *************************** //
            }
            else if( geoRep == GEO_REP_CAPSULE )  // capsule representation
            {
                fallShape = new btCapsuleShape( fabs(bulletstate->getPrimRadius()),
                    fabs(bulletstate->getPrimLength()) );
            }
            
            // ADDED BY CHRIS 2010-06-04 ******************************************** //
            else if(geoRep == GEO_REP_PLANE) // plane representation
            {    
                UT_Vector3 prim_t = bulletstate->getPrimT();
                UT_Vector3 prim_r = bulletstate->getPrimR();
                /*
                btScalar y = btCos(prim_r.z()) * btCos(prim_r.x());
                btScalar x = btCos(prim_r.z()) * btSin(prim_r.x());
                btScalar z = btSin(prim_r.z()) * btCos(prim_r.x());
                */
                //fallShape = new btStaticPlaneShape( btVector3( z,y,x ), prim_t.y() );
                fallShape = new btStaticPlaneShape( btVector3( 0,1,0 ), 0 );
            }
            
            // ADDED BY CHRIS 2010-06-23 ******************************************** //
            // Creates a cone primitive shape //
            
            else if(geoRep == GEO_REP_CONE_Y) // cone representation
            {
            	UT_Vector3 prim_s = bulletstate->getPrimS();
            	fallShape = new btConeShape( prim_s.x()/2, prim_s.y() );
            }


            // now add the shapes to bullet 
            if( fallShape )
            {
                // Physical Properties
                btScalar mass = 1.0f;
                btScalar restitution = 0.5;
                btScalar friction = 0.2;
                //btScalar linear_damping = 0;
                //btScalar angular_damping = 0;
                //btScalar linear = bulletstate->getLinearSleepThreshold();
                //btScalar angular = bulletstate->getAngularSleepThreshold();
                
                
                SIM_PhysicalParms *physicalparms = SIM_DATA_GET(*currObject,
                    "PhysicalParms", SIM_PhysicalParms);
                if( physicalparms )
                {
                    restitution = physicalparms->getBounce();
                    friction = physicalparms->getFriction();
                }
                
                
       
      
                // Calculate mass and inertia
                if( currObject->getIsStatic() )
                {
                    mass = 0;
                }
                else
                    mass = rbdstate->getMass();
                btVector3 fallInertia(0,0,0);
                fallShape->calculateLocalInertia(mass,fallInertia);
                   
                // collision margin / tolerance
                float collisionMargin = 0.0f;
                if( bulletstate )
                    collisionMargin = bulletstate->getCollisionMargin();
                fallShape->setMargin( collisionMargin );
                   
                btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(
                    mass,
                    fallMotionState,
                    //compoundShape,
                    fallShape,
                    fallInertia);
                fallRigidBodyCI.m_friction = friction;
                fallRigidBodyCI.m_restitution = restitution;
                
                //ADDED BY CHRIS
                //fallRigidBodyCI.m_linearSleepingThreshold = linear;
                //fallRigidBodyCI.m_angularSleepingThreshold = angular;
                
       
                // Initialize a new body
                sim_btRigidBody* fallRigidBody = new sim_btRigidBody(fallRigidBodyCI);
                state->m_dynamicsWorld->addRigidBody(fallRigidBody);
                //cout<<"creating new body, id:"<<currObject->getObjectId()
                //      <<"  isStaticObject:"<<fallRigidBody->isStaticObject()<<endl;
                   
                // ADDED BY SRH 2010-05-03 //
                //   Assign to the Bullet rigid body the ID of the corresponding Houdini object.
                fallRigidBody->houObjectId = currObject->getObjectId();
                // *********************** //
       
                // Insert the body into the map
                currBody.bodyId = fallRigidBody;
                currBody.isStatic =     false;
                currBody.dopId = currObject->getObjectId();
                currBody.type = "body";
                
                // CHRIS *** If it errors out at this point, it means you're not linking to the right bullet libraries/source files *** ///
                //cout<<"Is this where its erroring out again?"<<endl;
                state->m_bulletBodies->insert(std::make_pair( currObject->getObjectId(), currBody ));  
                bodyIt = state->m_bulletBodies->find( currObject->getObjectId() );
            }
        }      
    }
       
    //Return the body we found
    return bodyIt;

}




//Kill off the bullet bodies that are not needed anymore
void SIM_SnowSolverBullet::removeDeadBodies(SIM_Engine &engine)
{
    std::map< int, bulletBody >::iterator bodyIt;
    std::vector<int> deadBodies;
       
    // Accumulate all bodies to remove
    for( bodyIt = state->m_bulletBodies->begin(); bodyIt != state->m_bulletBodies->end(); ++bodyIt )
    {
        // ADDED BY SRH 2010-05-03 //
        //   Clear the last time step's collision manifolds for this Bullet rigid body.
        //   Collisions will be repopulated in the new time step.
        (bodyIt->second.bodyId)->m_manifolds.resize(0);
        // *********************** //
           
        SIM_Object *currObject = (SIM_Object *)engine.getSimulationObjectFromId(bodyIt->first);
        //This object no longer exists, destroy the bullet body (add to a map)
        if(!currObject)
        {
            //cout<<"removing bullet body, id:"<<currObject->getObjectId()<<endl;
            deadBodies.push_back(bodyIt->first);
        }
           
        // ADDED BY SRH 2010-04-27 //
        // If a body has been deactivated in Houdini (e.g. with an Active State node),
        //   this removes that body from the list of active objects
        /*else if ( currObject->getIsStatic() && (bodyIt->second.bodyId)->getInvMass() > 0. )
        {
            state->m_bulletBodies->erase(bodyIt);
        }*/
        // *********************** //
           
    }

    // ... remove them from bullet
    for(int b = 0; b < deadBodies.size(); b++)
    {
        bodyIt  = state->m_bulletBodies->find( deadBodies[b] );
        state->m_dynamicsWorld->removeRigidBody( (bodyIt->second.bodyId) );
        delete bodyIt->second.bodyId;
        //Erase map entry
        state->m_bulletBodies->erase(bodyIt);
    }
}


//Applies forces and constraints
//This loop could include more stuff since we're
//looping over all subData.. so may have to make it more
//general
void SIM_SnowSolverBullet::processSubData(SIM_Object *currObject, bulletBody &body)
{      
    int f;
    UT_String forcename;
    const SIM_Data *dopData;
       
    body.bodyId->clearForces();
       
    //Loop over subdata and get forces we're interested in
    for(f = 0; f< currObject->getNumSubData(); f++)
    {
               
        //Apply Uniform Force
        if(dopData = currObject->getNthConstSubData(
            &forcename, SIM_DataFilterByType("SIM_ForceUniform"), f, 0, SIM_DataFilterAll() ))
        {
            const SIM_ForceUniform *dopforce = SIM_DATA_CASTCONST( dopData, SIM_ForceUniform );
            UT_Vector3 thisforce = dopforce->getUniformForce();
            UT_Vector3 thistorque = dopforce->getUniformTorque();
            body.bodyId->applyCentralForce( btVector3(thisforce[0],thisforce[1],thisforce[2]) );
            //cout<<"adding central (uniform) force to id:"<<currObject->getObjectId()<<endl;
        }

        //Apply Gravity Force
        if(dopData = currObject->getNthConstSubData(
            &forcename, SIM_DataFilterByType("SIM_ForceGravity"), f, 0, SIM_DataFilterAll() ))
        {
            const SIM_ForceGravity *dopforce = SIM_DATA_CASTCONST( dopData, SIM_ForceGravity );
            UT_Vector3 thisforce = dopforce->getGravity();
            body.bodyId->setGravity(btVector3(thisforce[0],thisforce[1],thisforce[2]));
            //cout<<"adding gravity force"<<endl;
        }

        //Apply Point Force
        if(dopData = currObject->getNthConstSubData(
            &forcename, SIM_DataFilterByType("SIM_ForcePoint"), f, 0, SIM_DataFilterAll() ))
        {
            const SIM_ForcePoint *dopforce = SIM_DATA_CASTCONST( dopData, SIM_ForcePoint );
            UT_Vector3 thisforce = dopforce->getForce();
            UT_Vector3 thisPos = dopforce->getForcePosition();
            body.bodyId->applyForce( btVector3(thisforce[0],thisforce[1],thisforce[2]),
                btVector3(thisPos[0],thisPos[1],thisPos[2]) );
            //cout<<"adding (point) force to id:"<<currObject->getObjectId()<<endl;
        }

    }
}



void SIM_SnowSolverBulletState::initSystem(  )
{

    //cout<<"creating world: start"<<endl;
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new      btCollisionDispatcher( m_collisionConfiguration);
    m_broadphase = new btDbvtBroadphase();
#ifdef DO_SNOW_STUFF
    m_solver = new shMolecularDynamicsConstraintSolver();
#else
    m_solver = new btSequentialImpulseConstraintSolver();
#endif
       
#ifdef DO_SNOW_STUFF
    m_dynamicsWorld = new shGranularDiscreteDynamicsWorld(
        m_dispatcher,
        m_broadphase,
        m_solver,
        m_collisionConfiguration);
    //m_dynamicsWorld = world;
#else
    m_dynamicsWorld = new btDiscreteDynamicsWorld(
        m_dispatcher,
        m_broadphase,
        m_solver,
        m_collisionConfiguration);
#endif
       
    m_dynamicsWorld->clearForces();


    m_bulletBodies = new std::map<int, bulletBody>();
    m_bulletAffectors = new std::map<int, bulletBody>();

    // TEMP: add groundplane
    /*
    cout<<"creating temp groundplane"<<endl;
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(
        btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
    sim_btRigidBody* groundRigidBody = new sim_btRigidBody(groundRigidBodyCI);
    m_dynamicsWorld->addRigidBody(groundRigidBody);
    */
}

void SIM_SnowSolverBulletState::cleanSystem(  )
{
       
    if(  m_dynamicsWorld != NULL )
    {      
        if(  refCount == 0 )
        {      
            //cout<<"destroying world: start "<< m_dynamicsWorld<<endl;
            // delete in reverse order than creation
            for( int i =  m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i ) {
                btCollisionObject* obj =  m_dynamicsWorld->getCollisionObjectArray()[i];
                sim_btRigidBody* body = sim_btRigidBody::upcast( obj );
                if( body && body->getMotionState() )
                delete body->getMotionState();      
                m_dynamicsWorld->removeCollisionObject( obj );
                delete obj;
            }
           
            delete  m_dynamicsWorld;
            delete  m_solver;
            delete  m_broadphase;
            delete  m_dispatcher;
            delete  m_collisionConfiguration;
            m_collisionConfiguration = NULL;
            m_dispatcher = NULL;
            m_broadphase = NULL;
            m_solver = NULL;
            m_dynamicsWorld = NULL;
            m_bulletBodies->clear();
            delete  m_bulletBodies;
            //cout<<"destroying world: end"<<endl;
        }
    }

}



void initializeSIM(void *)
{
    //register our stuff with houdini
    //
    IMPLEMENT_DATAFACTORY(SIM_SnowSolverBullet);
    IMPLEMENT_DATAFACTORY(SIM_SnowBulletData);
    IMPLEMENT_DATAFACTORY(SIM_SnowNeighborData);
}


// ---------------------


const SIM_DopDescription*
SIM_SnowBulletData::getSnowBulletDataDopDescription()
{
    // ADDED BY CHRIS 2010-06-02 //
    //static PRM_Name            theChangeSleepThresholds(SIM_NAME_CHANGE_THRESHOLDS, "Change Sleeping Thresholds");
    
    static PRM_Default        defLinearSleep(0.1);
    static PRM_Default        defAngularSleep(1.0f);
    
    static PRM_Name            theLinearSleepThreshold(SIM_NAME_LINEAR_SLEEP_THRESHOLD, "Linear Sleeping Threshold");
    static PRM_Range        linearsleepthresholdRange(PRM_RANGE_UI, 0.0, PRM_RANGE_UI, 1.0);
    
    static PRM_Name            theAngularSleepThreshold(SIM_NAME_ANGULAR_SLEEP_THRESHOLD, "Angular Sleeping Threshold");
    static PRM_Range        angularsleepthresholdRange(PRM_RANGE_UI, 0.0, PRM_RANGE_UI, 1.0);
    
    //static PRM_Name            theAngularSleepThreshold(SIM_NAME_LINEAR_SLEEP_THRESHOLD, "Angular Sleeping Threshold");
    //static PRM_Name            theDeactivationTime(SIM_NAME_LINEAR_SLEEP_THRESHOLD, "Linear Sleeping Threshold");
    
    static PRM_Name            theIsKinematic(SIM_NAME_ISKINEMATIC, "Kinematic (Deforming Geometry)");
    
    // *********************** //

    static PRM_Name        theGeoRep(SIM_NAME_GEO_REP, "Geometry Representation");
    static PRM_Name        theGeoTri(SIM_NAME_GEO_TRI, "Triangulate Polygons (not working yet)");
    static PRM_Name        theGeoConvex(SIM_NAME_GEO_CONVEX, "Polygons As Convex Hulls");
    static PRM_Name        thePrimT(SIM_NAME_PRIM_T, "Prim Translate");
    static PRM_Name        thePrimR(SIM_NAME_PRIM_R, "Prim Rotate");
    static PRM_Name        thePrimS(SIM_NAME_PRIM_S, "Prim Scale");
    static PRM_Name        thePrimRadius(SIM_NAME_PRIM_RADIUS, "Prim Sphere Radius");
    static PRM_Name        thePrimLength(SIM_NAME_PRIM_LENGTH, "Prim Capsule Length");
    static PRM_Name        thePrimAutofit(SIM_NAME_PRIM_AUTOFIT, "AutoFit Primitive Spheres or Boxes to Geometry");
    static PRM_Name        theCollisionMargin(SIM_NAME_COLLISION_MARGIN, "Collision Tolerance");
    
    static PRM_Name        theGeoRepNames[] = {
        PRM_Name(GEO_REP_AS_IS,         "As Is"),
        PRM_Name(GEO_REP_SPHERE,        "Sphere"),
        PRM_Name(GEO_REP_BOX,           "Box"),
        PRM_Name(GEO_REP_CAPSULE,       "Capsule"),
        PRM_Name(GEO_REP_CONE_Y,		"Cone, Y-up"),
        PRM_Name(GEO_REP_PLANE,         "Ground Plane"),
        PRM_Name(0)
    };
    
    static PRM_ChoiceList   theGeoRepNamesMenu((PRM_ChoiceListType)(PRM_CHOICELIST_REPLACE | PRM_CHOICELIST_EXCLUSIVE ), &(theGeoRepNames[0]) );

    static PRM_Default         defZero(0);
    static PRM_Default         defOne(1);
    static PRM_Default         defGeoRep(0, GEO_REP_AS_IS);
    static PRM_Default         defCollisionMargin(0.0);
    static PRM_Default defZeroVec[]= {PRM_Default(0), PRM_Default(0), PRM_Default(0)};
    static PRM_Default defOneVec[] = {PRM_Default(1), PRM_Default(1), PRM_Default(1)};

    static PRM_Range           geoRepRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 5);
    static PRM_Range           geoGeoTriRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 1);
    static PRM_Range           primRadiusRange(PRM_RANGE_UI, 0.1, PRM_RANGE_UI, 5);
    static PRM_Range           primLengthRange(PRM_RANGE_UI, 0.1, PRM_RANGE_UI, 5);
    static PRM_Range           collisionMarginRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 0.5);
       
    static PRM_Template        theTemplates[] = {
        PRM_Template(PRM_STRING,        1, &theGeoRep, &defGeoRep, &theGeoRepNamesMenu),
        PRM_Template(PRM_TOGGLE_J,      1, &theGeoTri, &defOne),
        PRM_Template(PRM_TOGGLE_J,      1, &theGeoConvex, &defZero),
        PRM_Template(PRM_XYZ,           3, &thePrimT, defZeroVec),
        PRM_Template(PRM_XYZ,           3, &thePrimR, defZeroVec),
        PRM_Template(PRM_XYZ,           3, &thePrimS, defOneVec),
        PRM_Template(PRM_FLT_J,         1, &thePrimRadius, &defOne, 0, &primRadiusRange),
        PRM_Template(PRM_FLT_J,         1, &thePrimLength, &defOne, 0, &primLengthRange),
        PRM_Template(PRM_TOGGLE_J,      1, &thePrimAutofit, &defOne, 0, &geoGeoTriRange),
        PRM_Template(PRM_FLT_J,         1, &theCollisionMargin, &defCollisionMargin, 0, &collisionMarginRange),
        PRM_Template(PRM_FLT_J,         1, &theLinearSleepThreshold, &defLinearSleep, 0, &linearsleepthresholdRange),
        PRM_Template(PRM_FLT_J,         1, &theAngularSleepThreshold, &defAngularSleep, 0, &angularsleepthresholdRange),
        PRM_Template(PRM_TOGGLE_J,      1, &theIsKinematic, &defZero),
        PRM_Template()
    };
       
    static SIM_DopDescription   theDopDescription(true,
                                    "snowbulletdata",
                                    getName(),
                                    "Bullet Snow Data",
                                    classname(),
                                    theTemplates);
       
    return &theDopDescription;
}

const char* SIM_SnowBulletData::getName()
{
    static char name[256];
    sprintf (name, "Bullet Snow Data");
    return name;
}

SIM_SnowBulletData::SIM_SnowBulletData(const SIM_DataFactory *factory)
: BaseClass(factory), SIM_OptionsUser(this)
{
}

SIM_SnowBulletData::~SIM_SnowBulletData()
{
}








// ADDED BY SRH 2010-05-10 ********************************************************* //

// The SIM_SnowNeighborData defines a DOPs data type that
//   keeps track of which other objects a given DOPs object
//   is currently contacting against.

const SIM_DopDescription*
SIM_SnowNeighborData::getSnowNeighborDataDopDescription()
{
    static PRM_Name             theGeoNeighbors( SIM_NAME_GEO_NEIGHBORS, "Geometry Neighbors" );
    static PRM_Name             theNumNeighbors( SIM_NAME_NUM_NEIGHBORS, "Num Neighbors" );
    //static PRM_Name             theMaxStaticNeighbors( SIM_NAME_MAX_STATIC_NEIGHBORS, "Max Static Neighbors" );
    //static PRM_Name             theDeleteMe( SIM_NAME_DELETE_ME, "Delete Me" );
    
    static PRM_Default          defGeoNeighbors( 0, "[]" );
    static PRM_Default          defThree( 3 );
    static PRM_Default          defZero( 0 );
    
    static PRM_Template         theTemplates[] = {
        PRM_Template( PRM_STRING,       1, &theGeoNeighbors, &defGeoNeighbors ),
        PRM_Template( PRM_INT_J,        1, &theNumNeighbors, &defZero ),
        //PRM_Template( PRM_INT_J,        1, &theMaxStaticNeighbors, &defThree ),
        //PRM_Template( PRM_TOGGLE_J,     1, &theDeleteMe, &defZero ),
        PRM_Template()
    };
    
    static SIM_DopDescription    theDopDescription(true,
                                    "bulletneighbordata",
                                    getName(),
                                    "Bullet Neighbor Data",
                                    classname(),
                                    theTemplates);
    
    return &theDopDescription;
}

const char* SIM_SnowNeighborData::getName()
{
    static char name[256];
    sprintf (name, "Bullet Neighbor Data");
    return name;
}

SIM_SnowNeighborData::SIM_SnowNeighborData(const SIM_DataFactory *factory)
: BaseClass(factory), SIM_OptionsUser(this)
{
}

SIM_SnowNeighborData::~SIM_SnowNeighborData()
{
}

void SIM_SnowNeighborData::saveSubclass(ostream &os) const
{
    BaseClass::saveSubclass(os);
    saveOptionPacket(os, classname(), 0);
    int n = neighborIds.entries();
    UTwrite(os, &n);                                            // Write out size of the data array
    UTwrite(os, (const int *)neighborIds.getRawArray(), n);   // Write out data in the data array
}


bool SIM_SnowNeighborData::loadSubclass(UT_IStream &is)
{
    if (!BaseClass::loadSubclass(is))
    return false;

    if( !loadOptionPacket(is, classname(), 0) )
    return false;

    int n;
    bool ok = is.read(&n);              // Read in size of the data array
    if(ok)
    {
    neighborIds.resize(n);
    ok = is.read((int *)neighborIds.getRawArray(), n);    // Read in data from the data array
    }
    return ok;
}

SIM_Query* SIM_SnowNeighborData::createQueryObjectSubclass() const
{//cout << "querying object subclass" << endl;
    SIM_QueryArrays *query = new SIM_QueryArrays(this);
    // call addArray() for each field in your records
    query->addArray("Neighbors", "neighborid", &neighborIds);
    return new SIM_QueryCombine(BaseClass::createQueryObjectSubclass(), query);
}

void SIM_SnowNeighborData::makeEqualSubclass(const SIM_Data *source)
{
    const SIM_SnowNeighborData *p = (const SIM_SnowNeighborData*)source;
    BaseClass::makeEqualSubclass(source);
    // copy all custom values
    neighborIds = p->neighborIds;
}


// ********************************************************************************* //









void
SIM_SnowSolverBulletState::addReference()
{
    ++refCount;
}

void
SIM_SnowSolverBulletState::removeReference()
{
    --refCount;
    if (refCount < 1)
        delete this;
}

SIM_SnowSolverBulletState::SIM_SnowSolverBulletState() : refCount(0)
{
    initSystem();
}

SIM_SnowSolverBulletState::~SIM_SnowSolverBulletState()
{
    if( m_dynamicsWorld != NULL )
        cleanSystem();
}

