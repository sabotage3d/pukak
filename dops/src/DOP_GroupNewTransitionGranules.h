#ifndef GROUPNEWTRANSITIONGRANULES_H
#define GROUPNEWTRANSITIONGRANULES_H


#include <DOP/DOP_Node.h>
#include <DOP/DOP_Operator.h>

class DOP_GroupNewTransitionGranules : public DOP_Node
{
public:
             DOP_GroupNewTransitionGranules(OP_Network *net, const char *name,
                               OP_Operator *op);
    virtual ~DOP_GroupNewTransitionGranules();

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
    void            NEWGROUPNAME(UT_String &str, fpreal t);
    void            INTERIORGRANULESGROUPNAME( UT_String &str, float t );
//	void            EXTERIORGRANULESGROUPNAME( UT_String &str, float t );
};

#endif