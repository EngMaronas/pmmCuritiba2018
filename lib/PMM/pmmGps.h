#ifndef PMM_GPS_h
#define PMM_GPS_h

#include <pmmConsts.h>
#include <NMEAGPS.h>

typedef gps_fix Gps_structType;
class GpsManager
{
private:
    //------------------------------------------------------------
    // This object parses received characters
    //   into the gps.fix() data structure
    NMEAGPS mGps;

    //------------------------------------------------------------
    //  Define a set of GPS fix information.  It will hold on to the various pieces as they are received from
    //  an RMC sentence.  It can be used anywhere in your sketch.
    gps_fix mFix;

public:
    GpsManager();
    int init();
    int updateIfAvailable(gps_fix fix);
    void doSomeWork();
};

#endif
