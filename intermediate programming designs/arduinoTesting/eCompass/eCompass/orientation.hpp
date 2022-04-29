#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

class Orientation {
 public:
  static void calculate(float & roll, float & pitch, float & yaw, float & heading);
  static float yawToHeading(float yaw);
  static void hardIronCalibration(sensors_event_t accelEvent,sensors_event_t magEvent);
  static void reCalibrate(int n_iters);
  static void setParameters(
            Adafruit_LSM303_Accel_Unified * accl, Adafruit_LSM303_Mag_Unified * magn,
            float declination,
            float hardiron_x, float hardiron_y, float hardiron_z);
  

  static Adafruit_LSM303_Accel_Unified * accl;
  static Adafruit_LSM303_Mag_Unified * magn;
  static float declination;
  
  static float hardiron_x, hardiron_y, hardiron_z;
  static float AccelMinX, AccelMaxX, AccelMinY, AccelMaxY,AccelMinZ,AccelMaxZ;
  static float MagMinX, MagMaxX,MagMinY, MagMaxY,MagMinZ, MagMaxZ;

};

#endif
