#ifndef GPS_H
#define GPS_H

#include <SoftwareSerial.h>
#include <TinyGPS.h>

class GPS{
public:
    void updatePosition(float &lat, float &lon);
    float _heading;
private: 
    SoftwareSerial gpsSerial(PB1,PB0); //RX, TX 
    TinyGPS gps;
    /* internal heading can be used to calibrate heading from eCompass */
    float _lat, _lon;
    float updateHeading();
    float toRadians(float deg);
    float toDegrees(float rad);

    /* Used for return to base and updating internal heading */
    float headingBetween( float lat, float lon, float goalLat, float goalLon);
    float distanceBetween(float lat, float lon, float goalLat, float goalLon);
}

#endif