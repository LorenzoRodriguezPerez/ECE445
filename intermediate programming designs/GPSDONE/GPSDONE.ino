//Connect with pin 18 and 19
#include <TinyGPS.h>
//long   lat,lon; // create variable for latitude and longitude object
float lat,lon, _course, _knots;
TinyGPS gps; // create gps object

void setup(){
Serial.begin(9600); // connect serial
Serial.println("The GPS Received Signal:");
Serial2.begin(9600); // connect gps sensor

}
 
void loop(){
    while(Serial2.available()){ // check for gps data
    if(gps.encode(Serial2.read()))// encode gps data
    { 
    gps.f_get_position(&lat,&lon); // get latitude and longitude
    _course = gps.f_course();
    _knots = gps.f_speed_knots();
    //TODO: USE THESE LIBS INSTEAD!!
    //gps.course_to() 
    //gps.distance_between()
    Serial.print("Position: ");
    
    //Latitude
    Serial.print("Latitude: ");
    Serial.print(lat,6);
    
    Serial.print(",");
    
    //Longitude
    Serial.print("Longitude: ");
    Serial.print(lon,6); 

    Serial.print(",");
    
    Serial.print("Course: ");
    Serial.print(_course); 

    Serial.print(",");
    Serial.print("Knots: ");
    Serial.println(_knots); 
    
   }
  }
} 
