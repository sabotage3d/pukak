#ifndef DOP_MODIFYGEOMDATA
#define DOP_MODIFYGEOMDATA


#include <DOP/DOP_Node.h>


class DOP_GroupCollidingObjects : public DOP_Node
{
public:
             DOP_GroupCollidingObjects(OP_Network *net, const char *name,
                               OP_Operator *op);
    virtual ~DOP_GroupCollidingObjects();

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
    void            GROUP( UT_String &str, fpreal t );
    void            NEWGROUP( UT_String &str, float t );
    float           MINIMPULSE( float t );
    //int             INPUTINDEX(float t);
    /*
    float           VALUENUMBER( float t );
    void            VALUESTRING( UT_String &str, float t );
    void            GEOMATTRNAME( UT_String &str, float t );        // Get the attribute name for updating at time t
    int             GEOMATTRCLASS( float t );       // Get the attribute class (point or prim) at time t
    int             GEOMATTRTYPE( float t );        // Get the attribute type (int, float, or string) at time t
    */
};


#endif

