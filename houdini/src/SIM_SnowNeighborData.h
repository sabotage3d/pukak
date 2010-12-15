// The SIM_SnowNeighborData defines a DOPs data type that
//   keeps track of which other objects a given DOPs object
//   is currently contacting against.

#define SIM_NAME_GEO_NEIGHBORS          "geo_neighbors"
#define SIM_NAME_NUM_NEIGHBORS          "num_neighbors"
//#define SIM_NAME_MAX_STATIC_NEIGHBORS    "max_static_neighbors"
//#define SIM_NAME_DELETE_ME               "delete_me"

class SIM_SnowNeighborData : public SIM_Data,
                public SIM_OptionsUser
{
    public:
        GETSET_DATA_FUNCS_S( SIM_NAME_GEO_NEIGHBORS, GeoNeighbors );
        GETSET_DATA_FUNCS_I( SIM_NAME_NUM_NEIGHBORS, NumNeighbors );
        //GETSET_DATA_FUNCS_I( SIM_NAME_MAX_STATIC_NEIGHBORS, MaxStaticNeighbors );    // ADDED SRH 2010-06-09 - Keeps track of the max # of static neighbors a given granule can have before it is deleted
        //GETSET_DATA_FUNCS_B( SIM_NAME_DELETE_ME, DeleteMe );                         // ADDED SRH 2010-06-09 - Tell Houdini if this object should be culled out

        static const char* getName();
        
        // Data added to this array will be used for multiple data records.
        // Added by SRH 2010-05-29 //
        int getNumNeighborIds() const                    // For implementing multiple data records
            { return neighborIds.entries(); }
        int getNeighborId( int i ) const               // For implementing multiple data records
            { return neighborIds(i); }
        void appendNeighborId( float id )                // For implementing multiple data records
            { neighborIds.append(id); }
        void resetNeighborIds()
            { neighborIds.resize(0); }
        // *********************** //


    protected:
        // This ensures that this data is always kept in RAM, even when
        // the cache runs out of space and writes out the simulation
        // step to disk. This is necessary because the Bullet data can't
        // be written to disk.
        virtual bool getCanBeSavedToDiskSubclass() const
            { return false; }
           
        // This multiple records data should also be persisted so custom implementations
        //   of saveSubclass() and loadSubclass() should be provided
        // Added by SRH 2010-05-29
        virtual void saveSubclass(ostream &os) const;               // For implementing multiple data records
        virtual bool loadSubclass(UT_IStream &is);                  // For implementing multiple data records
        virtual SIM_Query *createQueryObjectSubclass() const;       // For implementing multiple data records
        virtual void makeEqualSubclass(const SIM_Data *source);     // For implementing multiple data records

        explicit         SIM_SnowNeighborData(const SIM_DataFactory *factory);
        virtual         ~SIM_SnowNeighborData();

    private:
        UT_IntArray neighborIds;
        
        static const SIM_DopDescription     *getSnowNeighborDataDopDescription();
    
        DECLARE_STANDARD_GETCASTTOTYPE();
        DECLARE_DATAFACTORY(SIM_SnowNeighborData,
                SIM_Data,
                "Bullet Neighbor Data",
                getSnowNeighborDataDopDescription()
        );
};