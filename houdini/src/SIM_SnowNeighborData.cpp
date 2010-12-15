#include "SIM_SnowNeighborData.h"

// The SIM_SnowNeighborData defines a DOPs data type that
//   keeps track of which other objects a given DOPs object
//   is currently contacting against.


void initializeSIM(void *)
{
    //register our stuff with houdini
    //
    IMPLEMENT_DATAFACTORY(SIM_SnowNeighborData);
}


const SIM_DopDescription*
SIM_SnowNeighborData::getSnowNeighborDataDopDescription()
{
    static PRM_Name             theGeoNeighbors( SIM_NAME_GEO_NEIGHBORS, "Geometry Neighbors" );
    static PRM_Name             theNumNeighbors( SIM_NAME_NUM_NEIGHBORS, "Num Neighbors" );
    //static PRM_Name             theMaxStaticNeighbors( SIM_NAME_MAX_STATIC_NEIGHBORS, "Max Static Neighbors" );
    //static PRM_Name             theDeleteMe( SIM_NAME_DELETE_ME, "Delete Me" );
    
    static PRM_Default          defGeoNeighbors( 0, "[]" );
    static PRM_Default          defThree( 3 );
    static PRM_Default          defZero( 0 );
    
    static PRM_Template         theTemplates[] = {
        PRM_Template( PRM_STRING,       1, &theGeoNeighbors, &defGeoNeighbors ),
        PRM_Template( PRM_INT_J,        1, &theNumNeighbors, &defZero ),
        //PRM_Template( PRM_INT_J,        1, &theMaxStaticNeighbors, &defThree ),
        //PRM_Template( PRM_TOGGLE_J,     1, &theDeleteMe, &defZero ),
        PRM_Template()
    };
    
    static SIM_DopDescription    theDopDescription(true,
                                    "bulletneighbordata",
                                    getName(),
                                    "Bullet Neighbor Data",
                                    classname(),
                                    theTemplates);
    
    return &theDopDescription;
}

const char* SIM_SnowNeighborData::getName()
{
    static char name[256];
    sprintf (name, "Bullet Neighbor Data");
    return name;
}

SIM_SnowNeighborData::SIM_SnowNeighborData(const SIM_DataFactory *factory)
: BaseClass(factory), SIM_OptionsUser(this)
{
}

SIM_SnowNeighborData::~SIM_SnowNeighborData()
{
}

void SIM_SnowNeighborData::saveSubclass(ostream &os) const
{
    BaseClass::saveSubclass(os);
    saveOptionPacket(os, classname(), 0);
    int n = neighborIds.entries();
    UTwrite(os, &n);                                            // Write out size of the data array
    UTwrite(os, (const int *)neighborIds.getRawArray(), n);   // Write out data in the data array
}


bool SIM_SnowNeighborData::loadSubclass(UT_IStream &is)
{
    if (!BaseClass::loadSubclass(is))
    return false;

    if( !loadOptionPacket(is, classname(), 0) )
    return false;

    int n;
    bool ok = is.read(&n);              // Read in size of the data array
    if(ok)
    {
    neighborIds.resize(n);
    ok = is.read((int *)neighborIds.getRawArray(), n);    // Read in data from the data array
    }
    return ok;
}

SIM_Query* SIM_SnowNeighborData::createQueryObjectSubclass() const
{//cout << "querying object subclass" << endl;
    SIM_QueryArrays *query = new SIM_QueryArrays(this);
    // call addArray() for each field in your records
    query->addArray( "Neighbors", "neighborid", &neighborIds );
    return new SIM_QueryCombine(BaseClass::createQueryObjectSubclass(), query);
}

void SIM_SnowNeighborData::makeEqualSubclass(const SIM_Data *source)
{
    const SIM_SnowNeighborData *p = (const SIM_SnowNeighborData*)source;
    BaseClass::makeEqualSubclass(source);
    // copy all custom values
    neighborIds = p->neighborIds;
}
