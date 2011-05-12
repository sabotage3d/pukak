

#ifndef SOP_GROUPEXTERIORPRIMS_H
#define SOP_GROUPEXTERIORPRIMS_H



#include <SOP/SOP_Node.h>


class SOP_GroupExteriorPrims : public SOP_Node
{
public:
    SOP_GroupExteriorPrims( OP_Network *net, const char *name, OP_Operator *op );
    virtual ~SOP_GroupExteriorPrims();

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
    UT_String INTPRIMGROUP(float t) {
        UT_String grp;
        evalString( grp, "intprimgroup", 0, t );
        return grp;
    }  // GROUP
    UT_String EXTPRIMGROUP(float t) {
        UT_String grp;
        evalString( grp, "extprimgroup", 0, t );
        return grp;
    }  // POINTGROUP
	double RADIUS(float t) {
		return (double)evalFloat( "radius", 0, t );
	}  // RADIUS
    
    /// This variable is used together with the call to the "checkInputChanged"
    /// routine to notify the handles (if any) if the input has changed.
    GU_DetailGroupPair      myDetailGroupPair;

    /// This is the group of geometry to be manipulated by this SOP and cooked
    /// by the method "cookInputGroups".
    const GB_PointGroup     *myGroup;
};



#endif