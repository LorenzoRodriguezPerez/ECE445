#ifndef ECOMPASS_H
#define ECOMPASS_H

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

class eCompass{
public:
    eCompass(float declination, Adafruit_LSM303_Accel_Unified * accl,Adafruit_LSM303_Mag_Unified * magn
    ,float hardiron_x = 0, float hardiron_y = 0, float hardiron_z = 0){
        _declination = declination;
        _accl = accl;
        _magn = magn;
        _hardiron_x = hardiron_x;
        _hardiron_y = hardiron_y;
        _hardiron_z = hardiron_z;
    }
    void calculate(float & roll, float & pitch, float & yaw, float & heading);
    void hardIronCalibration(int n_iters);

private:
    Adafruit_LSM303_Accel_Unified * _accl;
    Adafruit_LSM303_Mag_Unified * _magn;
    float _declination;
    float _hardiron_x, _hardiron_y, _hardiron_z;
    float accelMin_x, accelMax_x, accelMin_y, accelMax_y, accelMin_z,accelMax_z;
    float magMin_x, magMax_x, magMin_y,magMax_y,magMin_z,magMax_z;
    
    float yawToHeading(float yaw);

};

#endif
 