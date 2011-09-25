#ifndef DOP_CONSTRAINNEWTRANSITIONGRANULES
#define DOP_CONSTRAINNEWTRANSITIONGRANULES


#include <DOP/DOP_Node.h>

#include <DOP/DOP_Operator.h>


class DOP_ConstrainNewTransitionGranules : public DOP_Node
{
public:
             DOP_ConstrainNewTransitionGranules(OP_Network *net, const char *name,
                               OP_Operator *op);
    virtual ~DOP_ConstrainNewTransitionGranules();

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
    void            CONNECTEDGROUPSPREFIX( UT_String &str, float t );				// Gets the name prefix of the connected component groups
    void            CONSTRAINTOBJECTSPREFIX( UT_String &str, float t );				// Gets the name prefix of the objects that transition granules are constrained to
	void            GRANULEOBJECTSPREFIX( UT_String &str, float t );				// Gets the name prefix of the granule objects
	void            SIMULATECREATIONFRAME( int b, float t );						// Check whether the objects should simulate on the frame they were created
	void            SOLIDMESHGEOMETRYDATA( UT_String &str, float t );				// Name of the Geometry data for the solid mesh that is attached to the constraint object
	void            INTERIORGRANULEPOINTSGEOMETRYDATA( UT_String &str, float t );	// Name of the Geometry data for the current interior granules that is attached to the constraint object
};



class DOP_ConstrainNewTransitionGranulesOperator : public DOP_Operator
{
public:
	DOP_ConstrainNewTransitionGranulesOperator(const char *name, const char *english, OP_Constructor construct,
												PRM_Template *templates, unsigned min_sources, unsigned max_sources,
												CH_LocalVariable *variables, unsigned flags, unsigned num_outputs)
		: DOP_Operator( name, english, construct, templates, min_sources, max_sources, variables, flags, num_outputs )
	{}
	
	bool getHDKHelp( UT_String& str ) const
	{
		str.harden( "Group each interior granule with its surrounding shell granules into a group with the prefix indicated by the neighbor group prefix parameter and the suffix being the interior granule's objid.\n  This node will not do anything unless all parameters are filled in." );
		
		return true;
	}  // getHDKHelp()
};


#endif

