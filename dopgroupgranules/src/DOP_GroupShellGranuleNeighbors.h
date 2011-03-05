#ifndef DOP_GROUPSHELLGRANULENEIGHBORS
#define DOP_GROUPSHELLGRANULENEIGHBORS


#include <DOP/DOP_Node.h>


class DOP_GroupShellGranuleNeighbors : public DOP_Node
{
public:
             DOP_GroupShellGranuleNeighbors(OP_Network *net, const char *name,
                               OP_Operator *op);
    virtual ~DOP_GroupShellGranuleNeighbors();

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
    void            SHELLGRANULESGROUPNAME( UT_String &str, float t );
    void            INTERIORGRANULESGROUPNAME( UT_String &str, float t );
    void            NEIGHBORGROUPPREFIX( UT_String &str, float t );
};


#endif
