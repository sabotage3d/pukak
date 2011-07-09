#ifndef DOP_TRANSFEROBJIDTOMESH
#define DOP_TRANSFEROBJIDTOMESH


#include <DOP/DOP_Node.h>


class DOP_TransferObjidsToMesh : public DOP_Node
{
public:
             DOP_TransferObjidsToMesh(OP_Network *net, const char *name,
                               OP_Operator *op);
    virtual ~DOP_TransferObjidsToMesh();

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
    void            OBJECTSGROUP(UT_String &str, fpreal t);
    void            MESHOBJECTNAME( UT_String &str, float t );
};


#endif

