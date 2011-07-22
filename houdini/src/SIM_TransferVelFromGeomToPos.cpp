/*
 * SIM_TransferVelFromGeomToPos.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: Sethro
 */

#include <UT/UT_DSOVersion.h>
#include <UT/UT_XformOrder.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Options.h>
#include <SIM/SIM_Data.h>
#include <SIM/SIM_Utils.h>
#include <RBD/RBD_State.h>
#include <SIM/SIM_Engine.h>
#include <GEO/GEO_Detail.h>
#include <OP/OP_Node.h>

#include "SIM_TransferVelFromGeomToPos.h"

void initializeSIM(void *)
{
	//register our stuff with houdini
	//
	IMPLEMENT_DATAFACTORY(SIM_TransferVelFromGeomToPos);
}
//Constructor for the soft bullet body engine
SIM_TransferVelFromGeomToPos::SIM_TransferVelFromGeomToPos(const SIM_DataFactory *factory)
: BaseClass(factory), SIM_OptionsUser(this)
{
}

//Destructor for my soft bullet bodies
SIM_TransferVelFromGeomToPos::~SIM_TransferVelFromGeomToPos()
{
}


//get the UI for the bullet node
const SIM_DopDescription * SIM_TransferVelFromGeomToPos::getSolverTransferVelDopDescription()
{
	static PRM_Template            theTemplates[] = {
		PRM_Template()
	};

	static SIM_DopDescription   theDopDescription(true,
												  "transfervelfromgeomtopossolver",
												  "Vel From Geom to Pos Solver",
												  "Solver",
												  classname(),
												  theTemplates);

	return &theDopDescription;
}

//  The solveObjectsSubclass is called once per simulation step.
//  The objects array has all of the objects you are to solve for.
SIM_Solver::SIM_Result SIM_TransferVelFromGeomToPos::solveObjectsSubclass(SIM_Engine &engine,
																  SIM_ObjectArray &objects,
																  SIM_ObjectArray &newobjects,
																  SIM_ObjectArray &feedbacktoobjects,
																  const SIM_Time &timestep)
{
    int numObjects = newobjects.entries();
    for ( int i = 0; i < numObjects; i++ )
    {
        // Get the current object
        SIM_Object* currObject = (SIM_Object *)newobjects(i);
        
        // Get the current object's geometry data
        SIM_Geometry* myGeo = (SIM_Geometry *)currObject->getGeometry();
        GU_DetailHandleAutoReadLock     gdl(myGeo->getGeometry());
        GU_Detail* gdp = (GU_Detail *)gdl.getGdp();
        
        // Grab the velocity (v) from the geometry
        //GB_AttributeRef vAttrOffset = gdp->findPointAttrib( "v", sizeof(float)*3, GB_ATTRIB_VECTOR );
		GA_RWAttributeRef vAttrOffset = gdp->findFloatTuple( GA_ATTRIB_POINT, "v", 3 );	// Tuple size is 3
        GEO_Point* pt = gdp->points()(0);
        UT_Vector3 vel = pt->getValue<UT_Vector3>( vAttrOffset );
        
        // Assign the velocity to the Position data's vel
        RBD_State* pos = SIM_DATA_GET( *currObject, "Position", RBD_State );
        pos->setVelocity( vel );
    }  // for i
    
    return SIM_Solver::SIM_SOLVER_SUCCESS;
}  // solveObjectsSubclass()
