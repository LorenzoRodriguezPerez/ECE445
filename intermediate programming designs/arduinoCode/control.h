#ifndef CONTROL_H
#define CONTROL_H

#include <Servo.h>

#define MANUAL_MODE 0
#define AUTONOMOUS_MODE 1
#define RETURN_TO_BASE 2



class Control{
public:
    int _mode;
    int _rudderAngle, _winchAngle;
    
    Control(int winchChannel, int rudderChannel, int switch3Channel, int switchChannel, int rudderServo, int winchServo){
        WINCH = winchChannel;
        RUDDER = rudderChannel;
        SwC = switch3Channel;
        SwD = switchChannel;
        pinMode(WINCH, INPUT); 
        pinMode(RUDDER, INPUT);
        pinMode(SwC, INPUT);
        pinMode(SwD, INPUT);
        servos_init(rudderServo,winchServo);
    }


private: 
    /* rudderGimble, winchGimble, 3-way switch, set base switch */
    int WINCH, RUDDER, SwC, SwD;
    Servo _rudderServo; 
    Servo _winchServo;

    int readRudderGimble();
    int readWinchGimble();
    int read3Switch();
    int readSwitch();

    void setRudderAngle(int angle);
    void setWinchAngle(int angle);
}

#endif