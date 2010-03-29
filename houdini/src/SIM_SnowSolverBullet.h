
#ifndef __SIM_SnowSolverBullet_h__
#define __SIM_SnowSolverBullet_h__

#include <SIM/SIM_Solver.h>
#include <SIM/SIM_OptionsUser.h>

#include <map>
#include <vector>
#include <string>

#define SIM_NAME_SOURCEOBJECTS	"sourceobjects"
#define SIM_NAME_GRAVITY "gravity"
#define SIM_NAME_BOUNCE	"bounce"
#define SIM_NAME_FRICTION "friction"
#define SIM_NAME_ERP "erp"
#define SIM_NAME_CFM "cfm"
#define SIM_NAME_QUICK "quick"
#define SIM_NAME_OVERSAMPLE "oversample"
#define SIM_NAME_RAND "rand"
#define SIM_NAME_SHOWPRIM "showprim"

class SIM_ObjectArray;
class SIM_Geometry;
class SIM_GeometryCopy;



typedef struct bulletBodystr {
	UT_String type;
	int isStatic;
	int isSet;
	int dopId;
	btRigidBody* bodyId;
	UT_BoundingBox bbox;
} bulletBody;



#ifndef __SIM_SnowBulletData_h__
#define __SIM_SnowBulletData__
//#include "SIM/SIM_API.h"

#define SIM_NAME_GEO_REP             "geo_representation"
#define SIM_NAME_GEO_TRI             "geo_triangulate"
#define SIM_NAME_GEO_CONVEX          "geo_convexhull"
#define SIM_NAME_PRIM_T              "prim_t"
#define SIM_NAME_PRIM_R              "prim_r"
#define SIM_NAME_PRIM_S              "prim_s"
#define SIM_NAME_PRIM_RADIUS         "prim_radius"
#define SIM_NAME_PRIM_LENGTH         "prim_length"
#define SIM_NAME_PRIM_AUTOFIT        "prim_autofit"
#define SIM_NAME_PRIM_AUTOFIT_METHOD "prim_autofit_method"
#define SIM_NAME_COLLISION_MARGIN    "collision_margin"

#define GEO_REP_AS_IS		"as-is"
#define GEO_REP_SPHERE		"sphere"
#define GEO_REP_BOX		"box"
#define GEO_REP_CAPSULE		"capsule"
//#define GEO_REP_PITTON		"pitton"
//#define GEO_REP_SPHEREPACK	"spherepack"


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

		static const char* getName();

	protected:
		// This ensures that this data is always kept in RAM, even when
		// the cache runs out of space and writes out the simulation
		// step to disk. This is necessary because the Bullet data can't
		// be written to disk.
		virtual bool getCanBeSavedToDiskSubclass() const
			{ return false; }

	explicit             SIM_SnowBulletData(const SIM_DataFactory *factory);
	virtual             ~SIM_SnowBulletData();

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


class SIM_SnowSolverBulletState {
	public:
		int  refCount;
		std::map<int, bulletBody> *m_bulletBodies;
		btBroadphaseInterface*	m_broadphase;
		btCollisionDispatcher*	m_dispatcher;
		btConstraintSolver*	m_solver;
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


class SIM_SnowSolverBullet : public SIM_Solver, public SIM_OptionsUser
{
	public:
		SIM_SnowSolverBulletState       *state;

	protected:
		virtual void makeEqualSubclass(const SIM_Data *src)
		{
			//cout<<"makeequalSubclass called, start "<<endl;
			SIM_SnowSolverBullet *world;
			world = const_cast<SIM_SnowSolverBullet *> SIM_DATA_CASTCONST(src, SIM_SnowSolverBullet);
			if( world && world->state )
			{
				state = world->state;
				state->addReference();
			}
		}

		explicit SIM_SnowSolverBullet(const SIM_DataFactory *factory);
		virtual ~SIM_SnowSolverBullet();
		virtual SIM_Solver::SIM_Result solveObjectsSubclass(SIM_Engine &engine,
			SIM_ObjectArray &objects,
			SIM_ObjectArray &newobjects,
			SIM_ObjectArray &feedbacktoobjects,
			const SIM_Time &timestep);
		virtual std::map< int, bulletBody >::iterator SIM_SnowSolverBullet::addBulletBody(SIM_Object *currObject);
		virtual void SIM_SnowSolverBullet::removeDeadBodies(SIM_Engine &engine);
		virtual void SIM_SnowSolverBullet::processSubData(SIM_Object *currObject, bulletBody &body);

	private:
		static const SIM_DopDescription	*getSolverBulletDopDescription(); 
	
		DECLARE_STANDARD_GETCASTTOTYPE();
		DECLARE_DATAFACTORY(SIM_SnowSolverBullet,
			SIM_Solver,
			"Bullet Solver",
			getSolverBulletDopDescription());
};


#endif



