
void setup(){
Serial.begin(9600); // connect serial
Serial.println("The GPS Received Signal:");
Serial2.begin(9600); // connect gps sensor

}

/* +/-LAT = NORTH/SOUTH, +/-LONG = EAST/WEST */
/* TO SET UP: IF NO SIGNAL FOUND IN ~ 2 MIN SEND ERROR MESSAGE TO USER  
      (MISSION PLANNER)*/

/* TO DO: IF COORDINATES DO NOT CHANGE ~X AMOUNT THEN DO NOT TRUST SPEED FOR CALC */
void loop(){
  // check for gps data
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
