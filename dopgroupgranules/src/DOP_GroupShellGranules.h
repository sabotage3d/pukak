#ifndef DOP_GROUPSHELLGRANULES
#define DOP_GROUPSHELLGRANULES


#include <DOP/DOP_Node.h>


class DOP_GroupShellGranules : public DOP_Node
{
public:
             DOP_GroupShellGranules(OP_Network *net, const char *name,
                               OP_Operator *op);
    virtual ~DOP_GroupShellGranules();

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
    void            INTERIORGRANULESGROUPNAME( UT_String &str, float t );
    void            SHELLGRANULESGROUPNAME( UT_String &str, float t );
};


#endif

