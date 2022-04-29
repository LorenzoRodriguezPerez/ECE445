#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <eCompass.h>
#include <GPS.h>
#include <control.h>
#include <PID.h>
#include <boatStruct.h>

#define CH1 PA3 // RIGHT GIMBLE L/R, Winch Control   (CH1)
#define CH3 PA2 // LEFT GIMBLE U/D,  Rudder Control  (CH3)
#define CH5 PA0 // SwC, 0 = Manual, 1 = Autonomous, 2 = Return to Base (CH5)
#define CH6 PA1 // SwD, 0->1 = Set New Base Position                   (CH6)
#define RS  PB0 // Rudder Servo Pin
#define WS  PB1 // Winch Servo Pin

#define CALIBRATION 0       // SET TO 1 FOR CALIBRATION
#define CALIBRATION_ITERS 8 // SET # OF ROTATIONS TO CALIBRATE

Adafruit_LSM303_Accel_Unified * accl; 
Adafruit_LSM303_Accel_Unified * magn;
boatData* sailboat; 
eCompass compass; 
GPS gps;
Control sailControl;
PID sailAutonomous;

void setup() {
  if ( CALIBRATION ){

  }
  else {
    /* Declination in Champaign, Illinois : 3 25' */
    float declination = 3;
  
    sailboat = new boatData; 
    compass = eCompass(declination, accl, magn);
    gps = GPS();
    sailAutonomous = PID();
    gps.updatePosition(sailboat->lat,sailboat->lon);

    /* For now, updates base position on power on, 
    update to initialize on first user control input */
    sailboat->base_lat = sailboat->lat;
    sailboat->base_lon = sailboat->lon;

    sailControl = Control(CH1, CH3, CH5, CH6, RS, WS);

    /* Initialize Interrupts */
    attachInterrupt(digitalPinToInterrupt(CH5),sailControl.updateMode(), CHANGE);
    // TODO: see if interrupt works for rising (since PWM)
    attachInterrupt(digitalPinToInterrupt(CH6),gps.updatePosition(sailboat->base_lat,sailboat->base_lon), RISING);
  }
}

void loop() {
  // Perform Hard Iron Calibration, check GPS 
  if (CALIBRATION){
    compass.hardIronCalibration(CALIBRATION_ITERS);
  }

  else {

    if(sailControl._mode == MANUAL_MODE){
      while(sailControl._mode == MANUAL_MODE){
        sailControl.manual();
        // TODO: Update Sensor Data
        
      }
    }

    else if (sailControl._mode == AUTONOMOUS_MODE){
      // Save current heading -> desired heading to maintain
      float desiredHeading = sailboat->heading;
      boatData* screenshotBoat = new boatData; 

      while(sailControl._mode == AUTONOMOUS_MODE){
        // TODO: Update "sailboat" with Sensor Data Here

        // "Screenshot" current state of boat
        deepCopyBoat(sailboat,screenshotBoat);

        sailAutonomous.control(desiredHeading,screenshotBoat, sailControl);
      }
    }

    else {
      // User cannot update base position while in return to base mode:
      detachInterrupt(digitalPinToInterrupt(CH6));
      boatData* screenshotBoat = new boatData; 
      
      while(sailControl._mode == RETURN_TO_BASE){
        // TODO: Update "sailboat" with Sensor Data Here

        // "Screenshot" current state of boat
        deepCopyBoat(sailboat,screenshotBoat);

        // Do nothing if within 5 m of desired base
        float distanceToBase = gps.distanceBetween(screenshotBoat->lat,screenshotBoat->lon,screenshotBoat->base_lat,screenshotBoat->base_lon);
        if (distanceToBase > 5){
          float desiredHeading = gps.headingBetween(screenshotBoat->lat,screenshotBoat->lon,screenshotBoat->base_lat,screenshotBoat->base_lon);
          sailAutonomous.control(desiredHeading, screenshotBoat, sailControl);
        }
      }
      // Re-enable interrupt to set base position
      attachInterrupt(digitalPinToInterrupt(CH6),gps.updatePosition(sailboat->base_lat,sailboat->base_lon), RISING);
    }

  }

}

void debug(){
  Serial.println("......Sailboat......");
  Serial.print("Lat: ");
  Serial.print(sailboat->lat);
  Serial.print("  |Long: ");
  Serial.print(sailboat->lon);
  Serial.print("  |Heading: ");
  Serial.print(sailboat->heading);
  Serial.print("  |Yaw: ");
  Serial.print(sailboat->yaw);
  Serial.print("  |Pitch: ");
  Serial.print(sailboat->pitch);
  Serial.print("  |Roll: ");
  Serial.println(sailboat->roll);

  Serial.println("......Control......");
  Serial.print("Mode: ");
  Serial.print(sailControl._mode);
  Serial.print("  |Rudder Angle: ");
  Serial.print(sailControl._rudderAngle);
  Serial.print("  |Winch Angle: ");
  Serial.println(sailControl._winchAngle);

  Serial.println("......GPS......");
  Serial.print("Heading: ");
  Serial.println(gps._heading);
  delay(50000); 
}



