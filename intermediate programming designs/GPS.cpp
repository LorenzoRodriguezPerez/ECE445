#include <GPS.h> 
#include <cmath>
#define _USE_MATH_DEFINES

while(Serial2.available()){ 
    gps.encode(Serial2.read());
    if (gps.location.isUpdated()){
      Serial.print("LAT: ");  Serial.print(gps.location.lat(), 6);
      Serial.print(" | LONG:"); Serial.print(gps.location.lng(), 6);
      _course = gps.course.deg() ; _knots = gps.speed.knots(); _mph = gps.speed.mph();
      Serial.print(" | COURSE: ");Serial.print(_course);
      Serial.print(" | KNOTS: ");Serial.print(_knots);
      Serial.print(" | MPH: ");Serial.println(_mph);
      }
      
    //TODO: USE THESE LIBS INSTEAD
    //gps.courseTo(double lat1, double long1, double lat2, double long2)
    //gps.distanceBetween(double lat1, double long1, double lat2, double long2)
   
  }
} 

/*
void GPS::updateGPS(float &lat, float &lon){
    if(Serial2.available()){
        gps.encode(Serial2.read());
        _lat = gps.location.lat(); 
        _long = gps.location.long();
        _heading = gps.course.deg();
        _knots = gps.speed.knots();
        _mph = gps.speed.knots();
    }
    _lat = lat;
    _lon = lon;
}

void GPS::updateHeading(){
    float newLat, newLon;
    if(_gpsSerial.available()){
        if(gps.encode(_gpsSerial.read())){
            gps.f_get_position(&newLat,&newLon);
        }
    }
    if (_lat && _lon){
        _heading = headingBetween(_lat,_lon,newLat,newLon);
    }
    _lat = newLat;
    _lon = newLon;
}
*/
float GPS::toRadians(float degree) { return degree * (M_PI / 180); }
float GPS::toDegrees(float radian) { return radian * (180 / M_PI);}