

#ifndef SOP_CHOOSEFIRSTNEIGHBORGROUP_H
#define SOP_CHOOSEFIRSTNEIGHBORGROUP_H



#include <SOP/SOP_Node.h>


class SOP_ChooseFirstNeighborGroup : public SOP_Node
{
public:
    SOP_ChooseFirstNeighborGroup( OP_Network *net, const char *name, OP_Operator *op );
    virtual ~SOP_ChooseFirstNeighborGroup();

    static PRM_Template     myTemplateList[];
    static OP_Node          *myConstructor( OP_Network*, const char*, OP_Operator* );

    /// This method is created so that it can be called by handles.  It only
    /// cooks the input group of this SOP.  The geometry in this group is
    /// the only geometry manipulated by this SOP.
    ///virtual OP_ERROR        cookInputGroups( OP_Context &context, int alone = 0 );

protected:
    /// Method to cook geometry for the SOP
    virtual OP_ERROR        cookMySop( OP_Context &context );

private:
	UT_String NEIGHBORGROUPSMASK(float t) {
		UT_String mask;
		evalString( mask, "neighborgroupsmask", 0, t );
		return mask;
	}  // GROUP	
    UT_String MESHINTERIORGROUP(float t) {
        UT_String grp;
        evalString( grp, "meshinteriorgroup", 0, t );
        return grp;
    }  // ATTRIBNAME
	UT_String INTERIORGROUP(float t) {
        UT_String grp;
        evalString( grp, "interiorgroup", 0, t );
        return grp;
    }  // MATCHINGPOINTSGROUP
	UT_String CHOSENGROUPNAME(float t) {
		UT_String grpname;
		evalString( grpname, "chosengroupname", 0, t );
		return grpname;
	}  // CHOSENGROUPNAME
    
    /// This variable is used together with the call to the "checkInputChanged"
    /// routine to notify the handles (if any) if the input has changed.
    GU_DetailGroupPair      myDetailGroupPair;

    /// This is the group of geometry to be manipulated by this SOP and cooked
    /// by the method "cookInputGroups".
    //const GB_PointGroup     *myGroup;
	const GA_PointGroup     *myGroup;
};



#endif