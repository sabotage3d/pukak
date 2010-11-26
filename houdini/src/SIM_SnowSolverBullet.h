#ifndef __SIM_SnowSolverBullet_h__
#define __SIM_SnowSolverBullet_h__

#include <SIM/SIM_Solver.h>
#include <SIM/SIM_OptionsUser.h>

#include <map>
#include <vector>
#include <string>

#define SIM_NAME_SOURCEOBJECTS  "sourceobjects"
#define SIM_NAME_GRAVITY "gravity"
#define SIM_NAME_BOUNCE "bounce"
#define SIM_NAME_FRICTION "friction"
#define SIM_NAME_ERP "erp"
#define SIM_NAME_CFM "cfm"
#define SIM_NAME_QUICK "quick"
#define SIM_NAME_OVERSAMPLE "oversample"
#define SIM_NAME_RAND "rand"
#define SIM_NAME_SHOWPRIM "showprim"

#define CONSTRAINTTYPE_POINT2POINT      "point2point"


class SIM_ObjectArray;
class SIM_Geometry;
class SIM_GeometryCopy;




// ADDED BY SRH 2010-04-30 *************************************************** //
//   This class has additional data structures to help keep track of impact
//   information per rigid body, in order to be able to write out Houdini
//   Impacts data.
class sim_btRigidBody : public btRigidBody
{

public:
    
    btAlignedObjectArray<btPersistentManifold> m_manifolds;
    int houObjectId;
    
    
    // CONSTRUCTORS
    sim_btRigidBody( const btRigidBody::btRigidBodyConstructionInfo& constructionInfo ) : btRigidBody( constructionInfo )
    {
        houObjectId = -1;
    }  // CONSTRUCTOR
    
    sim_btRigidBody( btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia=btVector3(0,0,0) )
        : btRigidBody( mass, motionState, collisionShape, localInertia )
    {
        houObjectId = -1;
    }  // CONSTRUCTOR
    
    
        static const sim_btRigidBody* upcast(const btCollisionObject* colObj)
    {
        if ( colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY )
            return (const sim_btRigidBody*)colObj;
        return 0;
    }  // upcast()
       
    static sim_btRigidBody* upcast(btCollisionObject* colObj)
    {
        if ( colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY )
            return (sim_btRigidBody*)colObj;
        return 0;
    }  // upcast()
       

};  // class sim_btRigidBody
// *************************************************************************** //





typedef struct bulletBodystr {
    UT_String type;
    int isStatic;
    int isSet;
    int dopId;
    sim_btRigidBody* bodyId;
    UT_BoundingBox bbox;
    // ADDED BY SRH 2010-08-10 //
    //   Constraint stuff
    //std::vector<SIM_ConAnchor*> anchors;
    //std::vector<btPoint2PointConstraint*> btPoint2PointConstraints;
    //std::vector<btHingeConstraint*> btHingeConstraints;
    // *********************** //
} bulletBody;




typedef struct bulletConstraintstr {
    UT_String type;
    btTypedConstraint* constraint;
    SIM_ConAnchor* anchor;
    //SIM_ConAnchor* anchorGoal;
    const SIM_Relationship* rel;
    SIM_Object* obj0;
    SIM_Object* obj1;
} bulletConstraint;




#ifndef __SIM_SnowBulletData_h__
#define __SIM_SnowBulletData__
//#include "SIM/SIM_API.h"

#define SIM_NAME_GEO_REP                "geo_representation"
#define SIM_NAME_GEO_TRI                "geo_triangulate"
#define SIM_NAME_GEO_CONVEX             "geo_convexhull"
#define SIM_NAME_PRIM_T                 "prim_t"
#define SIM_NAME_PRIM_R                 "prim_r"
#define SIM_NAME_PRIM_S                 "prim_s"
#define SIM_NAME_PRIM_RADIUS            "prim_radius"
#define SIM_NAME_PRIM_LENGTH            "prim_length"
#define SIM_NAME_PRIM_AUTOFIT           "prim_autofit"
#define SIM_NAME_PRIM_AUTOFIT_METHOD    "prim_autofit_method"
#define SIM_NAME_COLLISION_MARGIN       "collision_margin"

#define GEO_REP_AS_IS           "as-is"
#define GEO_REP_SPHERE          "sphere"
#define GEO_REP_BOX             "box"
#define GEO_REP_CAPSULE         "capsule"
#define GEO_REP_CONE_Y          "cone, y-up"
#define GEO_REP_PLANE        "groundplane"
#define GEO_REP_CYLINDER_Y	"cylinder, y-up"
//#define GEO_REP_PITTON        "pitton"
//#define GEO_REP_SPHEREPACK    "spherepack"

//#define SIM_NAME_CHANGE_THRESHOLDS        "change_thresholds"
#define SIM_NAME_LINEAR_SLEEP_THRESHOLD     "linear_sleep_threshold"
#define SIM_NAME_ANGULAR_SLEEP_THRESHOLD    "angular_sleep_threshold"
#define SIM_NAME_ISKINEMATIC                "is_kinematic"
//#define DO_SNOW_STUFF   1




class SIM_SnowBulletData : public SIM_Data,
                public SIM_OptionsUser
{
    public:
        GETSET_DATA_FUNCS_S(SIM_NAME_GEO_REP, GeoRep);
        GETSET_DATA_FUNCS_B(SIM_NAME_GEO_TRI, GeoTri);
        GETSET_DATA_FUNCS_B(SIM_NAME_GEO_CONVEX, GeoConvex);
        GETSET_DATA_FUNCS_V3(SIM_NAME_PRIM_T, PrimT);
        GETSET_DATA_FUNCS_V3(SIM_NAME_PRIM_R, PrimR);
        GETSET_DATA_FUNCS_V3(SIM_NAME_PRIM_S, PrimS);
        GETSET_DATA_FUNCS_F(SIM_NAME_PRIM_RADIUS, PrimRadius);
        GETSET_DATA_FUNCS_F(SIM_NAME_PRIM_LENGTH, PrimLength);
        GETSET_DATA_FUNCS_F(SIM_NAME_COLLISION_MARGIN, CollisionMargin);
        GETSET_DATA_FUNCS_B(SIM_NAME_PRIM_AUTOFIT, Autofit);
        GETSET_DATA_FUNCS_I(SIM_NAME_PRIM_AUTOFIT_METHOD, AutofitMethod);
        
        //Added by Chris 2010-06-07 //
        //GETSET_DATA_FUNCS_B(SIM_NAME_CHANGE_THRESHOLDS, ChangeSleepThresholds);       	// Indicates if user wants to use custom sleeping thresholds, rather than the default ones.
        GETSET_DATA_FUNCS_F(SIM_NAME_LINEAR_SLEEP_THRESHOLD, LinearSleepThreshold);     	// Sets the linear sleeping threshold value
        GETSET_DATA_FUNCS_F(SIM_NAME_ANGULAR_SLEEP_THRESHOLD, AngularSleepThreshold);      	// Sets the linear sleeping threshold value
        // *********************** //
        
        //Added by SRH 2010-06-10//
        GETSET_DATA_FUNCS_B(SIM_NAME_ISKINEMATIC, IsKinematic);    				//Indicates whether or not bullet should re-evaluate the static geometry; i.e. if you have animated static meshes.
        
        
        static const char* getName();

    protected:
        // This ensures that this data is always kept in RAM, even when
        // the cache runs out of space and writes out the simulation
        // step to disk. This is necessary because the Bullet data can't
        // be written to disk.
        virtual bool getCanBeSavedToDiskSubclass() const
            { return false; }

    explicit         SIM_SnowBulletData(const SIM_DataFactory *factory);
    virtual         ~SIM_SnowBulletData();

    private:
        static const SIM_DopDescription     *getSnowBulletDataDopDescription();
       
        DECLARE_STANDARD_GETCASTTOTYPE();
        DECLARE_DATAFACTORY(SIM_SnowBulletData,
                SIM_Data,
                "Bullet Snow Data",
                getSnowBulletDataDopDescription()
            );
};
#endif






// ADDED BY SRH 2010-05-10 ********************************************************* //

// The SIM_SnowNeighborData defines a DOPs data type that
//   keeps track of which other objects a given DOPs object
//   is currently contacting against.

#define SIM_NAME_GEO_NEIGHBORS          "geo_neighbors"
#define SIM_NAME_NUM_NEIGHBORS          "num_neighbors"
//#define SIM_NAME_MAX_STATIC_NEIGHBORS    "max_static_neighbors"
//#define SIM_NAME_DELETE_ME               "delete_me"

class SIM_SnowNeighborData : public SIM_Data,
                public SIM_OptionsUser
{
    public:
        GETSET_DATA_FUNCS_S( SIM_NAME_GEO_NEIGHBORS, GeoNeighbors );
        GETSET_DATA_FUNCS_I( SIM_NAME_NUM_NEIGHBORS, NumNeighbors );
        //GETSET_DATA_FUNCS_I( SIM_NAME_MAX_STATIC_NEIGHBORS, MaxStaticNeighbors );    // ADDED SRH 2010-06-09 - Keeps track of the max # of static neighbors a given granule can have before it is deleted
        //GETSET_DATA_FUNCS_B( SIM_NAME_DELETE_ME, DeleteMe );                         // ADDED SRH 2010-06-09 - Tell Houdini if this object should be culled out

        static const char* getName();
        
        // Data added to this array will be used for multiple data records.
        // Added by SRH 2010-05-29 //
        int getNumNeighborIds() const                    // For implementing multiple data records
            { return neighborIds.entries(); }
        int getNeighborId( int i ) const               // For implementing multiple data records
            { return neighborIds(i); }
        void appendNeighborId( float id )                // For implementing multiple data records
            { neighborIds.append(id); }
        void resetNeighborIds()
            { neighborIds.resize(0); }
        // *********************** //


    protected:
        // This ensures that this data is always kept in RAM, even when
        // the cache runs out of space and writes out the simulation
        // step to disk. This is necessary because the Bullet data can't
        // be written to disk.
        virtual bool getCanBeSavedToDiskSubclass() const
            { return false; }
           
        // This multiple records data should also be persisted so custom implementations
        //   of saveSubclass() and loadSubclass() should be provided
        // Added by SRH 2010-05-29
        virtual void saveSubclass(ostream &os) const;               // For implementing multiple data records
        virtual bool loadSubclass(UT_IStream &is);                  // For implementing multiple data records
        virtual SIM_Query *createQueryObjectSubclass() const;       // For implementing multiple data records
        virtual void makeEqualSubclass(const SIM_Data *source);     // For implementing multiple data records

        explicit         SIM_SnowNeighborData(const SIM_DataFactory *factory);
        virtual         ~SIM_SnowNeighborData();

    private:
        UT_IntArray neighborIds;
        
        static const SIM_DopDescription     *getSnowNeighborDataDopDescription();
    
        DECLARE_STANDARD_GETCASTTOTYPE();
        DECLARE_DATAFACTORY(SIM_SnowNeighborData,
                SIM_Data,
                "Bullet Neighbor Data",
                getSnowNeighborDataDopDescription()
        );
};
// ********************************************************************************* //








class SIM_SnowSolverBulletState {
    public:
        int  refCount;
        std::map<int, bulletBody> *m_bulletBodies;
        std::map<string, bulletConstraint> *m_bulletConstraints;
        // ADDED BY SRH 2010-03-31 //
        std::map<int, bulletBody> *m_bulletAffectors;
        // *********************** //
        btBroadphaseInterface*  m_broadphase;
        btCollisionDispatcher*  m_dispatcher;
        btConstraintSolver*     m_solver;
#ifdef DO_SNOW_STUFF
        shGranularDiscreteDynamicsWorld* m_dynamicsWorld;
#else
        btDiscreteDynamicsWorld* m_dynamicsWorld;
#endif
        btDefaultCollisionConfiguration* m_collisionConfiguration;
        SIM_SnowSolverBulletState();
        ~SIM_SnowSolverBulletState();
        void addReference();
        void removeReference();
        void initSystem( );
        void cleanSystem(  );
};





// ADDED BY SRH 2010-06-02 //
#define SIM_NAME_COMPUTE_IMPACTS     "compute_impacts"
#define SIM_NAME_SUBSTEPS            "substeps"

// *********************** //


class SIM_SnowSolverBullet : public SIM_Solver, public SIM_OptionsUser
{
    public:
        SIM_SnowSolverBulletState       *state;
           
        // ADDED BY SRH 2010-06-02 //
        GETSET_DATA_FUNCS_B(SIM_NAME_COMPUTE_IMPACTS, ComputeImpacts);  // Mark whether or not Bullet should compute impacts data (default on)
        GETSET_DATA_FUNCS_I(SIM_NAME_SUBSTEPS, Substeps);       // Set number of substeps the Bullet Engine will take, additional to the DOP Network's substeps setting
        // *********************** //
        

        

    protected:
        virtual void makeEqualSubclass(const SIM_Data *src)
        {
            SIM_SnowSolverBullet *world;
            world = const_cast<SIM_SnowSolverBullet *> SIM_DATA_CASTCONST(src, SIM_SnowSolverBullet);
            if( world && world->state )
            {
                state = world->state;
                state->addReference();
            }
            //bob = world->bob;
        }
        
        //int bob;

        explicit SIM_SnowSolverBullet(const SIM_DataFactory *factory);
        virtual ~SIM_SnowSolverBullet();
        virtual SIM_Solver::SIM_Result solveObjectsSubclass(SIM_Engine &engine,
            SIM_ObjectArray &objects,
            SIM_ObjectArray &newobjects,
            SIM_ObjectArray &feedbacktoobjects,
            const SIM_Time &timestep);
        virtual std::map< int, bulletBody >::iterator addBulletBody(SIM_Object *currObject);
        virtual void removeDeadBodies(SIM_Engine &engine);
        virtual void processSubData(SIM_Object *currObject, bulletBody &body);

    private:
        static const SIM_DopDescription *getSolverBulletDopDescription();
       
        DECLARE_STANDARD_GETCASTTOTYPE();
        DECLARE_DATAFACTORY(SIM_SnowSolverBullet,
            SIM_Solver,
            "Bullet Solver",
            getSolverBulletDopDescription());
};


#endif

