#ifndef SIM_GROUPOBJECTS_H
#define SIM_GROUPOBJECTS_H


#include <SIM/SIM_Solver.h>
#include <SIM/SIM_OptionsUser.h>




#define SIM_NAME_GROUP_NAME                 "group_name"
#define SIM_NAME_DO_COLLIDING_OBJECTS       "do_colliding_objects"


class SIM_GroupObjects : public SIM_Solver,
                public SIM_OptionsUser
{
public:
    GETSET_DATA_FUNCS_S(SIM_NAME_GROUP_NAME, GroupName);
    GETSET_DATA_FUNCS_B(SIM_NAME_DO_COLLIDING_OBJECTS, DoCollidingObjects);
        
    static const char* getName();

protected:
    // This ensures that this data is always kept in RAM, even when
    // the cache runs out of space and writes out the simulation
    // step to disk. This is necessary because the Bullet data can't
    // be written to disk.
    virtual bool getCanBeSavedToDiskSubclass() const
        { return false; }

    explicit SIM_GroupObjects(const SIM_DataFactory *factory);
    virtual  ~SIM_GroupObjects();
    virtual  SIM_Solver::SIM_Result solveObjectsSubclass(SIM_Engine &engine,
                                                         SIM_ObjectArray &objects,
                                                         SIM_ObjectArray &newobjects,
                                                         SIM_ObjectArray &feedbacktoobjects,
                                                         const SIM_Time &timestep);

private:
    static const SIM_DopDescription     *getGroupObjectsDopDescription();
    
    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_GroupObjects,
                        SIM_Solver,
                        "Group Solver",
                        getGroupObjectsDopDescription()
    );
};


#endif