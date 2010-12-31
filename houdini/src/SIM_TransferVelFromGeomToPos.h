/*
 * SIM_TransferVelFromGeomToPos.h
 *
 *  Created on: Sep 30, 2010
 *      Author: Owen Hancock
 */


#include <SIM/SIM_Solver.h>
#include <SIM/SIM_OptionsUser.h>

#include <map>
#include <vector>
#include <string>




#ifndef SIM_TRANSFERVELFROMGEOMTOPOS_H_
#define SIM_TRANSFERVELFROMGEOMTOPOS_H_


class SIM_TransferVelFromGeomToPos: public SIM_Solver,
				public SIM_OptionsUser
{
public:


protected:
	explicit SIM_TransferVelFromGeomToPos(const SIM_DataFactory * factory);
	virtual ~SIM_TransferVelFromGeomToPos();

	virtual SIM_Solver::SIM_Result solveObjectsSubclass(SIM_Engine &engine,
															SIM_ObjectArray &objects,
															SIM_ObjectArray &newobjects,
															SIM_ObjectArray &feedbacktoobjects,
															const SIM_Time &timestep);

private:
	static const SIM_DopDescription *getSolverTransferVelDopDescription();

	DECLARE_STANDARD_GETCASTTOTYPE();
	DECLARE_DATAFACTORY(SIM_TransferVelFromGeomToPos,
						SIM_Solver,
						"Vel From Geom to Pos Solver",
						getSolverTransferVelDopDescription());

};

#endif /* SIM_SOLVERSOFTBULLET_H_ */
