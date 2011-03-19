#ifndef SOP_IMPORTCONNECTEDINTERIORGRANULES_H
#define SOP_IMPORTCONNECTEDINTERIORGRANULES_H



#include <SOP/SOP_Node.h>


class SOP_ImportConnectedInteriorGranules : public SOP_Node
{
public:
    SOP_ImportConnectedInteriorGranules( OP_Network *net, const char *name, OP_Operator *op );
    virtual ~SOP_ImportConnectedInteriorGranules();

    static PRM_Template     myTemplateList[];
    static OP_Node          *myConstructor(OP_Network*, const char*, OP_Operator* );

    /// This method is created so that it can be called by handles.  It only
    /// cooks the input group of this SOP.  The geometry in this group is
    /// the only geometry manipulated by this SOP.
    ///virtual OP_ERROR        cookInputGroups( OP_Context &context, int alone = 0 );

protected:
    /// Method to cook geometry for the SOP
    virtual OP_ERROR        cookMySop( OP_Context &context );

private:
    //int RADIUS() { return evalInt( "rad", 0, 0 ); }
    //int NUMPOINTS() { return evalInt( "numpoints", 0, 0 ); }
    UT_String DOPPATH(float t) {
        UT_String path;
        evalString( path, "doppath", 0, t );
        return path;
    }  // DOPPATH
    UT_String GROUPMASK(float t) {
        UT_String mask;
        evalString( mask, "groupmask", 0, t );
        return mask;
    }  // GROUPMASK
    UT_String INTERIORGRANULESGROUP(float t) {
        UT_String intgranulesgrp;
        evalString( intgranulesgrp, "intgranulesgrp", 0, t );
        return intgranulesgrp;
    }  // GROUPMASK
    
    UT_Vector4 computeChildPosition( UT_Vector4 p1, UT_Vector4 p2, UT_Vector4 norm, fpreal radius );
    bool intersectRaySphere( UT_Vector4 rayOrigin, UT_Vector4 ray, UT_Vector4 sphCenter, fpreal radius ); 
    
    //const GB_PointGroup*    parsePointGroups(const char *pattern, GU_Detail  which_gdp=0, int allow_numeric=1);

    /// This variable is used together with the call to the "checkInputChanged"
    /// routine to notify the handles (if any) if the input has changed.
    GU_DetailGroupPair      myDetailGroupPair;

    /// This is the group of geometry to be manipulated by this SOP and cooked
    /// by the method "cookInputGroups".
    const GB_PointGroup     *myGroup;
};



#endif