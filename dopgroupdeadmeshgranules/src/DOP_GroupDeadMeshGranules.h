#ifndef DOP_MODIFYGEOMDATA
#define DOP_MODIFYGEOMDATA


#include <DOP/DOP_Node.h>


class DOP_GroupDeadMeshGranules : public DOP_Node
{
public:
             DOP_GroupDeadMeshGranules(OP_Network *net, const char *name,
                               OP_Operator *op);
    virtual ~DOP_GroupDeadMeshGranules();

    static OP_Node              *myConstructor(OP_Network *net,
                                               const char *name,
                                               OP_Operator *op);
    static PRM_Template          myTemplateList[];

protected:
    virtual void         processObjectsSubclass(fpreal time,
                                        int foroutputidx,
                                        const SIM_ObjectArray &objects,
                                        DOP_Engine &engine);
    virtual void         getInputInfoSubclass(int inputidx,
                                        DOP_InOutInfo &info);
    virtual void         getOutputInfoSubclass(int inputidx,
                                        DOP_InOutInfo &info);

private:
    void            GROUP(UT_String &str, fpreal t);
    void            NEWGROUPNAME( UT_String &str, float t );
    void            MESHNAME( UT_String &str, float t );
};


#endif

