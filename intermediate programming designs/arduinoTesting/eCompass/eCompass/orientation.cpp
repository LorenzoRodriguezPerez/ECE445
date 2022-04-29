#include "orientation.hpp"

#define RAD_CONV 57.2957 // 180/Pi, to convert radians to degrees

float Orientation::hardiron_x, Orientation::hardiron_y, Orientation::hardiron_z;
float Orientation::AccelMinX, Orientation::AccelMaxX, Orientation::AccelMinY, Orientation::AccelMaxY, Orientation::AccelMinZ, Orientation::AccelMaxZ;
float Orientation::MagMinX, Orientation::MagMaxX, Orientation::MagMinY, Orientation::MagMaxY, Orientation::MagMinZ, Orientation::MagMaxZ;

float Orientation::declination;
Adafruit_LSM303_Accel_Unified * Orientation::accl;
Adafruit_LSM303_Mag_Unified * Orientation::magn;

/** Use an eCompass algorithm to calculate orientation
 * 
 * The algorithm is based on http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
 * 
 * Hardiron calibration must be performed. The process is simple:
 *   1. Mount the magnetometer in the location that you intend to use it at
 *   2. Rotate the body through all possible orientations
 *   3. Record the minimum and maximum for each axis of the magnetometer
 *   4. Average the minumum and maximum for each axis. This will give you your hardiron x,y,z offsets.
 *
 * @param accl The LSM303 acceleration object
 * @param magn The LSM303 magnetometer object
 * @param hardiron_x
 * @param hardiron_y
 * @param hardiron_z
 * @param roll The result roll in degrees
 * @param pitch The result pitch in degrees
 * @param yaw The result yaw in degrees
 */
void Orientation::calculate(float & roll, float & pitch, float & yaw, float & heading){
  sensors_event_t event_accl; 
  sensors_event_t event_magn; 

  accl->getEvent(&event_accl); 
  magn->getEvent(&event_magn);

  //hardIronCalibration(event_accl,event_magn);

  // Signs choosen so that, when axis is down, the value is + 1g
  float accl_x = -event_accl.acceleration.x;
  float accl_y = event_accl.acceleration.y;
  float accl_z = event_accl.acceleration.z;

  // Signs should be choosen so that, when the axis is down, the value is + positive.
  // But that doesn't seem to work ?...
  float magn_x = event_magn.magnetic.x - hardiron_x;
  float magn_y = -event_magn.magnetic.y - hardiron_y;
  float magn_z = -event_magn.magnetic.z - hardiron_z;  
  
  // Freescale solution
  roll = atan2(accl_y, accl_z);
  pitch = atan(-accl_x / (accl_y * sin(roll) + accl_z * cos(roll)));
  
  float magn_fy_fs = magn_z * sin(roll) - magn_y*cos(roll);
  float magn_fx_fs = magn_x * cos(pitch) + magn_y * sin(pitch) * sin(roll) + magn_z * sin(pitch) * cos(roll);
  
  yaw = atan2(magn_fy_fs, magn_fx_fs);
  
  roll = roll * RAD_CONV;
  pitch = pitch * RAD_CONV;
  yaw = yaw * RAD_CONV;
  
  heading = yawToHeading(yaw);
}

/** Convert a yaw (-180 to 180) to a compass heading (0 to 360) with declination correction
 * 
 * @param yaw The input yaw, in degrees -180 to 180
 * @param declination the magnetic declination to adjust against, with sign. Note that this is time and location dependent.
 * @return the corrected heading, in degrees 0 to 360
 */
float Orientation::yawToHeading(float yaw){
    float heading = yaw + declination;
    if (heading < 0.0){
        heading += 360.0;
    }
    return heading;
}
void Orientation::reCalibrate(int n_iters){
    AccelMinX = 0;
    AccelMaxX = 0;
    AccelMinY = 0;
    AccelMaxY = 0;
    AccelMinZ = 0;  
    AccelMaxZ = 0;
    MagMinX = 0;
    MagMaxX = 0;
    MagMinY = 0; 
    MagMaxY = 0;
    MagMinZ = 0;
    MagMaxZ = 0;
    for (int i =0;i<n_iters;i++){
        sensors_event_t event_accl; 
        sensors_event_t event_magn; 

        accl->getEvent(&event_accl); 
        magn->getEvent(&event_magn);
        hardIronCalibration(event_accl, event_magn);
    }

}
void Orientation::hardIronCalibration(sensors_event_t accelEvent, sensors_event_t magEvent) 
{
  
  if (accelEvent.acceleration.x < AccelMinX) AccelMinX = accelEvent.acceleration.x;
  if (accelEvent.acceleration.x > AccelMaxX) AccelMaxX = accelEvent.acceleration.x;
  
  if (accelEvent.acceleration.y < AccelMinY) AccelMinY = accelEvent.acceleration.y;
  if (accelEvent.acceleration.y > AccelMaxY) AccelMaxY = accelEvent.acceleration.y;

  if (accelEvent.acceleration.z < AccelMinZ) AccelMinZ = accelEvent.acceleration.z;
  if (accelEvent.acceleration.z > AccelMaxZ) AccelMaxZ = accelEvent.acceleration.z;

  if (magEvent.magnetic.x < MagMinX) MagMinX = magEvent.magnetic.x;
  if (magEvent.magnetic.x > MagMaxX) MagMaxX = magEvent.magnetic.x;
  
  if (magEvent.magnetic.y < MagMinY) MagMinY = magEvent.magnetic.y;
  if (magEvent.magnetic.y > MagMaxY) MagMaxY = magEvent.magnetic.y;

  if (magEvent.magnetic.z < MagMinZ) MagMinZ = magEvent.magnetic.z;
  if (magEvent.magnetic.z > MagMaxZ) MagMaxZ = magEvent.magnetic.z;

  hardiron_x = (MagMaxX + MagMinX) / 2;
  hardiron_y = (MagMaxY + MagMinY) / 2;
  hardiron_z = (MagMaxZ + MagMinZ) / 2;
}

void Orientation::setParameters(
    Adafruit_LSM303_Accel_Unified * new_accl, Adafruit_LSM303_Mag_Unified * new_magn,
    float new_declination,
    float new_hardiron_x, float new_hardiron_y, float new_hardiron_z){
    accl = new_accl;
    magn = new_magn;
    declination = new_declination;
    hardiron_x = new_hardiron_x;
    hardiron_y = new_hardiron_y;
    hardiron_z = new_hardiron_z;
    AccelMinX = 0;
    AccelMaxX = 0;
    AccelMinY = 0;
    AccelMaxY = 0;
    AccelMinZ = 0;  
    AccelMaxZ = 0;
    MagMinX = 0;
    MagMaxX = 0;
    MagMinY = 0; 
    MagMaxY = 0;
    MagMinZ = 0;
    MagMaxZ = 0;
}
