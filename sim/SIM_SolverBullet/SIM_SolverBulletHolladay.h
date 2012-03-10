/*
 * Copyright (c) 2011
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SIM_SolverBullet_h__
#define __SIM_SolverBullet_h__

#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Solver.h>

#include <SIM/SIM_ConRel.h>

#define SIM_NAME_SUBSTEPS  "substeps"
#define SIM_NAME_ITERATION  "numiteration"
#define SIM_NAME_SPLITIMPULSE "splitimpulse"
#define SIM_NAME_SIPT  "penetrationthreshold"

class simBulletState;

// This is the Bullet Solver class, inherited from a Solver and the Options 
// for the Solver which are attached as Options.
class SIM_SolverBulletHolladay : public SIM_Solver, public SIM_OptionsUser
{
public:
    // methods to get/set the solver's Options
    GETSET_DATA_FUNCS_F(SIM_NAME_SUBSTEPS, Substeps);
    GETSET_DATA_FUNCS_I(SIM_NAME_ITERATION, NumIteration);
    GETSET_DATA_FUNCS_I(SIM_NAME_SPLITIMPULSE, SplitImpulse);
    GETSET_DATA_FUNCS_F(SIM_NAME_SIPT, SplitImpulsePenetrationThreshold);

protected:
    explicit SIM_SolverBulletHolladay(const SIM_DataFactory *factory);
    virtual ~SIM_SolverBulletHolladay();

    virtual void makeEqualSubclass(const SIM_Data *src);

    virtual SIM_Result solveObjectsSubclass(SIM_Engine &engine,
					SIM_ObjectArray &objects,
					SIM_ObjectArray &newobjects,
					SIM_ObjectArray &feedbacktoobjects,
					const SIM_Time &timestep);

private:
    static const SIM_DopDescription *getSolverBulletDopDescription(); 

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_SolverBulletHolladay, SIM_Solver, "Bullet Solver Holladay",
			getSolverBulletDopDescription());

    // the Bullet simulation
    mutable simBulletState *myState;
};









//  show the geo representation
#define SIM_NAME_GEO_REP	    "bullet_georep"
#define SIM_NAME_GEO_CONVEX	    "bullet_geoconvexhull"
#define SIM_NAME_PRIM_AUTOFIT	    "bullet_autofit"
#define SIM_NAME_PRIM_T		    "bullet_primT"
#define SIM_NAME_PRIM_R		    "bullet_primR"
#define SIM_NAME_PRIM_S		    "bullet_primS"
#define SIM_NAME_PRIM_RADIUS	    "bullet_radius"
#define SIM_NAME_PRIM_LENGTH	    "bullet_length"
#define SIM_NAME_ADJUST_GEOMETRY    "bullet_adjust_geometry"
#define SIM_NAME_ADJUST_FACTOR	    "bullet_adjust_factor"
#define SIM_NAME_COLLISION_MARGIN   "bullet_collision_margin"
#define SIM_NAME_ADDIMPACT	    "bullet_add_impact"
#define SIM_NAME_DEACTIVATE	    "bullet_want_deactivate"
#define SIM_NAME_OPTIMIZED	    "bullet_optimized"

#define GEO_REP_AS_IS		    "as-is"
#define	GEO_REP_COMPOUND	    "compound"
#define GEO_REP_SPHERE		    "sphere"
#define GEO_REP_BOX		    "box"
#define GEO_REP_CAPSULE		    "capsule"

class SIM_Geometry;

// Sim Bullet Dataset which is attached to each dop-object
class SIM_BulletData : public SIM_Data,	public SIM_OptionsUser
{
public:
    GETSET_DATA_FUNCS_S(SIM_NAME_GEO_REP, GeoRep);
    GETSET_DATA_FUNCS_B(SIM_NAME_GEO_CONVEX, GeoConvex);
    GETSET_DATA_FUNCS_B(SIM_NAME_PRIM_AUTOFIT, Autofit);
    GETSET_DATA_FUNCS_V3(SIM_NAME_PRIM_T, PrimT);
    GETSET_DATA_FUNCS_V3(SIM_NAME_PRIM_R, PrimR);
    GETSET_DATA_FUNCS_V3(SIM_NAME_PRIM_S, PrimS);
    GETSET_DATA_FUNCS_F(SIM_NAME_PRIM_RADIUS, PrimRadius);
    GETSET_DATA_FUNCS_F(SIM_NAME_PRIM_LENGTH, PrimLength);
    GETSET_DATA_FUNCS_B(SIM_NAME_ADJUST_GEOMETRY, AdjustGeometry);
    GETSET_DATA_FUNCS_F(SIM_NAME_ADJUST_FACTOR, AdjustFactor);
    GETSET_DATA_FUNCS_F(SIM_NAME_COLLISION_MARGIN, CollisionMargin);
    GETSET_DATA_FUNCS_B(SIM_NAME_ADDIMPACT, AddImpact);
    GETSET_DATA_FUNCS_B(SIM_NAME_DEACTIVATE, Deactivate);
    GETSET_DATA_FUNCS_B(SIM_NAME_OPTIMIZED, Optimized);

    void autofit(const SIM_Geometry &simgeo);

protected:
    explicit	 SIM_BulletData(const SIM_DataFactory *factory);
    virtual	~SIM_BulletData();

    // guide geometry
    virtual SIM_Guide	*createGuideObjectSubclass() const;
    virtual void	 buildGuideGeometrySubclass(const SIM_RootData &root,
						    const SIM_Options &options,
						    const GU_DetailHandle &gdh,
						    UT_Matrix4D *xform,
						    const SIM_Time &t) const;

    virtual bool getIsAlternateRepresentationSubclass() const { return true; }

private:
    static const SIM_DopDescription *getBulletDataDopDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_BulletData, SIM_Data, "Bullet Data",
			getBulletDataDopDescription());
};










#define	SIM_NAME_SWING1	"swing_span_1"
#define	SIM_NAME_SWING2	"swing_span_2"
#define	SIM_NAME_TWIST	"twist_span_"
#define SIM_NAME_SOFTNESS "softness"
#define SIM_NAME_BIAS_FACTOR "bias_factor"
#define SIM_NAME_RELAXATION_FACTOR "relaxation_factor"
#define SIM_NAME_TWIST_AXIS "twisting_axis"

// This class only provides get and set data functions, the creation of the 
// cone twist constraint is done by the solver, just like all other types of
// constraint.
class SIM_ConRelConeTwist : public SIM_ConRel, public SIM_OptionsUser
{
public:
    GETSET_DATA_FUNCS_F(SIM_NAME_SWING1, SwingSpan1);
    GETSET_DATA_FUNCS_F(SIM_NAME_SWING2, SwingSpan2);
    GETSET_DATA_FUNCS_F(SIM_NAME_TWIST, TwistSpan);
    GETSET_DATA_FUNCS_F(SIM_NAME_SOFTNESS, Softness);
    GETSET_DATA_FUNCS_F(SIM_NAME_BIAS_FACTOR, BiasFactor);
    GETSET_DATA_FUNCS_F(SIM_NAME_RELAXATION_FACTOR, RelaxationFactor);
    GETSET_DATA_FUNCS_V3(SIM_NAME_TWIST_AXIS, TwistAxis);

protected:
    explicit	 SIM_ConRelConeTwist(const SIM_DataFactory *factory);
    virtual	~SIM_ConRelConeTwist();
    
private:
    static const SIM_DopDescription *getConRelConeTwistDopDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_ConRelConeTwist, 
			SIM_ConRel, 
			"Cone Twist Constraint Relationship",
			getConRelConeTwistDopDescription());
};



#endif
