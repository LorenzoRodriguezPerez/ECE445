#include <control.h>
/*
#define WINCH PA3 // RIGHT GIMBLE L/R, Winch Control   (CH1)
#define RUDDER PA2 // LEFT GIMBLE U/D,  Rudder Control  (CH3)
#define SwC PA0 // SwC, 0 = Manual, 1 = Autonomous, 2 = Return to Base (CH5)
#define SwD PA1 // SwD, 0->1 = Set New Base Position                   (CH6)
*/

void Control::updateMode(){
    noInterrupts(); // Critical Section

    int pwm = pulseIn(SwC, HIGH);

    // Manual Mode Enabled
    if (pwm < 1450) {
        _mode = MANUAL_MODE;
    }
    
    // Autonomous Mode Enabled
    else if (pwm < 1950) {
        _mode = AUTONOMOUS_MODE;
    }

    // Return to Base Mode Enabled
    else {
        _mode = RETURN_TO_BASE;
    }
    interrupts();
}

void Control::manual(){
    int rudderAngle = readRudderGimble(RUDDER);
    int winchAngle = readWinchGimble(WINCH);
    setRudderAngle(rudderAngle);
    setWinchAngle(min(max(winchAngle + ch1Value,0),180));
}

void Control::servos_init(){
    _rudderServo.attach(rudderServo);
    _winchServo.attach(winchServo);
    setRudderAngle(90);
    setWinchAngle(90);
}

void setRudderAngle(int angle){
    _rudderServo.write(angle);
    _rudderAngle = angle;
}

void setWinchAngle(int angle){
    _winchServo.write(angle);
    _winchAngle = angle;
}

// Setup interrupts instead:
/*
Control::updateInput(){
    ch5Value = read3Switch(CH5);
    set_mode(ch5Value);
    ch6Value = readSwitch(CH6);
    set_base(ch6Value);
    ch3Value = readRudderGimble(CH3);
    ch1Value = readWinchGimble(CH1);
}
*/
// Maybe setup interrupt for changing mode only? 
// but still want to take input for when in manual... 

// If the channel is off, return the default value
/*
int Control::readChannel(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  int angle = map(pwm,1000,2000,0,180);
  return angle;
}
*/

// Left Gimble U/D
int Control::readRudderGimble(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  return map(pwm,1000,2000,0,180);
}

// Right Gimble L/R
int Control::readWinchGimble(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  int phi = floor(( 1500 - pwm ) / 25);
  return phi;
}

//SwC
/*
int Control::read3Switch(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  if (pwm < 1450) return 0;
  else if (pwm < 1950) return 1;
  else return 2;
} */


// SwD
// may have to use if rising interrupt does not work
/*
bool Control::readSwitch(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  if (pwm < 1450) return 0;
  else return 1;
}
*/