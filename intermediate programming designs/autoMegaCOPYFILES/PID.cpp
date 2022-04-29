#include "PID.hpp"

/* Returns rudder angle from PID control in range -45 -> 45 */
double PID::getRudderAngle(float deviation, float period){
    //float derivative = (deviation - prevDeviation) / period;
    /*
    float derivative = (deviation + (13*prevDeviation[0]) + (77*prevDeviation[1]) + (273*prevDeviation[2]) + (637*prevDeviation[3]) + (1001*prevDeviation[4]) + (1001*prevDeviation[5]) 
        + (429*prevDeviation[6]) - (429* prevDeviation[7]) - (1001* prevDeviation[8]) - (1001* prevDeviation[9]) - (637* prevDeviation[10]) - (273* prevDeviation[11]) - (77* prevDeviation[12]) - (13* prevDeviation[13])  - prevDeviation[14]) / (16384 * period);
    */
    float integral  = prevIntegral + (deviation * period);
    //double newAngle = (Kp * deviation) + (Kd * derivative) + (Ki * integral);
    
    double newAngle = Kp*deviation + (Ki * integral);
    
    // Check if saturated, if yes recalculate
    if (newAngle >= 45 || newAngle <= - 45 ){
        integral  = prevIntegral;
    }

    prevIntegral = integral;
    //prevDeviation = deviation;
    /*
    prevDeviation[14] = prevDeviation[13];
    prevDeviation[13] = prevDeviation[12];
    prevDeviation[12] = prevDeviation[11];
    prevDeviation[11] = prevDeviation[10];
    prevDeviation[10] = prevDeviation[9];
    prevDeviation[9] = prevDeviation[8];
    prevDeviation[8] = prevDeviation[7];
    prevDeviation[7] = prevDeviation[6];
    prevDeviation[6] = prevDeviation[5];
    prevDeviation[5] = prevDeviation[4];
    prevDeviation[4] = prevDeviation[3];
    prevDeviation[3] = prevDeviation[2];
    prevDeviation[2] = prevDeviation[1];
    prevDeviation[1] = prevDeviation[0];
    prevDeviation[0] = deviation;*/
    
    return newAngle;
    //return 45;
    
}
