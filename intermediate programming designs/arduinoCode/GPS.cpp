#include <GPS.h> 
#include <cmath>
#define _USE_MATH_DEFINES

void GPS::updatePosition(float &lat, float &lon){
    if(gpsSerial.available()){
        if(gps.encode(gpsSerial.read())){
            gps.f_get_position(&lat,&lon);
        }
    }
    _lat = lat;
    _lon = lon;
}

void GPS::updateHeading(){
    float newLat, newLon;
    if(gpsSerial.available()){
        if(gps.encode(gpsSerial.read())){
            gps.f_get_position(&newLat,&newLon);
        }
    }
    if (_lat && _lon){
        _heading = headingBetween(_lat,_lon,newLat,newLon);
    }
    _lat = newLat;
    _lon = newLon;
}

float GPS::toRadians(float degree) { return degree * (M_PI / 180); }
float GPS::toDegrees(float radian) { return radian * (180 / M_PI);}

// https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
// all values need to be passed in as RADIANS
float GPS::headingBetween(float lat, float lon, float goalLat, float goalLon){
    
    float x = cos(toRadians(goalLat)) * sin(toRadians(goalLon - lon));
    float y = (cos(toRadians(lat)) * sin(toRadians(goalLat))) - (sin(toRadians(lat)) * cos(toRadians(goalLat)) * cos(toRadians(goalLon - lon)));
    float beta = atan2(x,y);

    return toDegrees(beta);
}

// https://www.movable-type.co.uk/scripts/latlong.html
float GPS::distanceBetween(float lat, float lon, float goalLat, float goalLon){
    float R = 6371000; // Earth's radius (m)
    float phi = toRadians(goalLat - lat);
    float lambda = toRadians(goalLon - lon);

    /* Haversine */
    float a = pow(sin(phi/2),2) + cos(toRadians(lat)) * cos(toRadians(goalLat)) * pow(sin(lambda/2),2);
    float c = 2 * atan2(sqrt(a),sqrt(1-a));
    return R * c; // (m)
}