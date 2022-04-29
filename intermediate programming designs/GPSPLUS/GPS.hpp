#ifndef GPS_H
#define GPS_H

// INSTEAD: if(Serial2.available()){gps.encode(Serial2.read());}


//Connect with pin 18 and 19 TO DO IN MAIN LOOP ?
// Serial2.begin(9600); // connect gps sensor
#include <TinyGPS++.h>
TinyGPSPlus gps;


/* +/-LAT = NORTH/SOUTH, +/-LONG = EAST/WEST */
/* TO SET UP: IF NO SIGNAL FOUND IN ~ 2 MIN SEND ERROR MESSAGE TO USER  
      (MISSION PLANNER)*/

/* TO DO: IF COORDINATES DO NOT CHANGE ~X AMOUNT THEN DO NOT TRUST SPEED FOR CALC */

class GPS{
public:
    void updatePosition(float &lat, float &lon);
    /* Used for return to base and updating internal heading */
    float headingBetween( float lat, float lon, float goalLat, float goalLon);
    float distanceBetween(float lat, float lon, float goalLat, float goalLon);
private: 
    //SoftwareSerial _gpsSerial(PB1,PB0); //RX, TX 
    TinyGPSPlus gps;
    /* internal heading can be used to calibrate heading from eCompass */
    float _lat, _lon, _heading, _knots, _mph;
    float updateHeading();
    float toRadians(float deg);
    float toDegrees(float rad);

};

#endif
