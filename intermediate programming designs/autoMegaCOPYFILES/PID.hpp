#ifndef PID_HPP
#define PID_HPP

class PID{
public:
    PID() {
        prevIntegral = 0;
    }
    static float prevIntegral;
    static double getRudderAngle(float deviation, float period);
private: 
    static long Kp = 1.5;//5.2539;//2.1;//5.2539;
    static float Td = 0.1;
    static float Ti = 3;
    static float Ki = Kp / Ti;
    static float Kd = Kp * Td;
    //float prevDeviation[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    // NEED FREQ OF GIVING "NEW" RUDDER ANGLE
};

#endif
