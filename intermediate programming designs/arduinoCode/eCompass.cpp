#include <eCompass.h>
#include <unistd.h>

void eCompass::calculate(float & roll, float & pitch, float & yaw, float & heading){
    sensors_event_t event_accl, event_magn;

    _accl->getEvent(&event_accl); 
    _magn->getEvent(&event_magn);
    
    float accl_x = -event_accl.acceleration.x;
    float accl_y = event_accl.acceleration.y;
    float accl_z = event_accl.acceleration.z;

    // Possibly have to adjust +/- 
    float magn_x = event_magn.magnetic.x - hardiron_x;
    float magn_y = -event_magn.magnetic.y - hardiron_y;
    float magn_z = -event_magn.magnetic.z - hardiron_z;  
  
    // (Taken from Freescale)
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

/*
 * Hardiron calibration must be performed. The process is simple:
 *   1. Mount the magnetometer in the location that you intend to use it at
 *   2. Rotate the body through all possible orientations
 *   3. Record the minimum and maximum for each axis of the magnetometer
 *   4. Average the minumum and maximum for each axis. This will give you your hardiron x,y,z offsets.
 * TODO: determine n_iters, maybe 8 -> rotate boat every 45 degrees 
*/
void eCompass::hardIronCalibration(int n_iters){
    sensors_event_t accelEvent; 
    sensors_event_t magEvent; 
    for (int i = 0; i < n_iters; i++){
        accl->getEvent(&event_accl); 
        magn->getEvent(&event_magn);
        if (accelEvent.acceleration.x < accelMin_x) accelMin_x = accelEvent.acceleration.x;
        if (accelEvent.acceleration.x > accelMax_x) accelMax_x = accelEvent.acceleration.x;
  
        if (accelEvent.acceleration.y < accelMin_y) accelMin_y = accelEvent.acceleration.y;
        if (accelEvent.acceleration.y > accelMax_y) accelMax_y = accelEvent.acceleration.y;

        if (accelEvent.acceleration.z < accelMin_z) accelMin_z = accelEvent.acceleration.z;
        if (accelEvent.acceleration.z > accelMax_z) accelMax_z = accelEvent.acceleration.z;

        if (magEvent.magnetic.x < magMin_x) magMin_x = magEvent.magnetic.x;
        if (magEvent.magnetic.x > magMax_x) magMax_x = magEvent.magnetic.x;
  
        if (magEvent.magnetic.y < magMin_y) magMin_y = magEvent.magnetic.y;
        if (magEvent.magnetic.y > magMax_y) magMax_y = magEvent.magnetic.y;

        if (magEvent.magnetic.z < magMin_z) magMin_z = magEvent.magnetic.z;
        if (magEvent.magnetic.z > magMax_z) magMax_z = magEvent.magnetic.z;
        
        // rotate Sailboat every 3 seconds
        sleep(3);
    }
    _hardiron_x = (magMax_x + magMin_x) / 2;
    _hardiron_y = (magMax_y + magMin_y) / 2;
    _hardiron_z = (magMax_z + magMin_z) / 2;
}


float eCompass::yawToHeading(float yaw){
    float heading = yaw + _declination;
    if (heading < 0.0){
        heading += 360.0;
    }
    return heading;
}
