#include<Arduino.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Vector3D.h>
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

void displaySensorDetails(void) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) {
  Serial.begin(115200);
  //Serial.println("Magnetometer Test");
  //Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }

  #ifndef ESP8266
  while (!Serial)
    ; // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Accelerometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
      /* Display some basic information on this sensor */
  displaySensorDetails();
  
  accel.setRange(LSM303_RANGE_4G);
  Serial.print("Range set to: ");
  lsm303_accel_range_t new_range = accel.getRange();
  switch (new_range) {
  case LSM303_RANGE_2G:
    Serial.println("+- 2G");
    break;
  case LSM303_RANGE_4G:
    Serial.println("+- 4G");
    break;
  case LSM303_RANGE_8G:
    Serial.println("+- 8G");
    break;
  case LSM303_RANGE_16G:
    Serial.println("+- 16G");
    break;
  }

  accel.setMode(LSM303_MODE_NORMAL);
  Serial.print("Mode set to: ");
  lsm303_accel_mode_t new_mode = accel.getMode();
  switch (new_mode) {
  case LSM303_MODE_NORMAL:
    Serial.println("Normal");
    break;
  case LSM303_MODE_LOW_POWER:
    Serial.println("Low Power");
    break;
  case LSM303_MODE_HIGH_RESOLUTION:
    Serial.println("High Resolution");
    break;
   }
 }
 
    
    Vector3D operator* (double x, const Vector3D& y)
    {
        return (y * x);
    }
    
double computeYaw(double mag_x, double mag_y, double mag_z, double accel_x, double accel_y, double accel_z)
    {
        const Vector3D vector_mag(mag_x, mag_y, mag_z);
        const Vector3D vector_down(accel_x, accel_y, accel_z);
        const Vector3D vector_north = vector_mag - ((vector_mag.dot(vector_down) / vector_down.dot(vector_down)) * vector_down);
        return atan2(vector_north.getX(), vector_north.getY()) * 180 / M_PI;
    }
void loop(void) {
  /* Get a new sensor event */
  /*If the X Gauss data is 0, check to see if the Y Gauss data is less than 0. 
   * If Y is less than 0 Gauss, the direction D is 90 degrees; 
   * 
   if Y is greater than or equal to 0 Gauss, the direction D is 0 degrees.
  */
  sensors_event_t event;
  mag.getEvent(&event);
  /* Get acceleration event */
  sensors_event_t event2;
  accel.getEvent(&event2);
  
  float Pi = 3.14159;
  // Calculate the angle of the vector y,x
  float heading = atan2(event.magnetic.y, event.magnetic.x) * (180 / Pi);
  //heading = atan2(y, x) * 180 / M_PI;
  float heading2 = computeYaw(event.magnetic.x,event.magnetic.y, event.magnetic.z, event2.acceleration.x, event2.acceleration.y,event2.acceleration.z);
  // Normalize to 0-360
  //if (heading < 0) {
    //heading = 360 + heading;
  //}
  float heading3;
  if (event.magnetic.x == 0){
    if (event.magnetic.y < 0){
      heading3 = 90;
    }
    else heading3 = 0;
  }
  else{
    heading3 = heading;
  }
  
  
  Serial.print("Heading:");
  Serial.print(heading);
  Serial.print("   Heading2:");
  Serial.print(heading2);
  Serial.print("   Adjusted_Heading:");
  Serial.print(heading3);


  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("  X: ");
  Serial.print(event2.acceleration.x);
  Serial.print("  Y: ");
  Serial.print(event2.acceleration.y);
  Serial.print("  Z: ");
  Serial.print(event2.acceleration.z);
  Serial.println("   m/s^2");
  
  delay(500);
}
