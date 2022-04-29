#include <Servo.h>

// Define Input Connections
#define CH1 PA3 
#define CH3 PA2
#define CH5 PA0
#define CH6 PA1

 // CHANNEL 1: RIGHT JOY L/R
 // CHANNEL 2: RIGHT JOY U/D
 // CHANNEL 3: LEFT JOY U/D
 // CHANNEL 4: LEFT JOY L/R
 // CHANNEL 5: SWC
 // CHANNEL 6: SWD

 // TOLERANCE FOR JOYS +- 2 ? 
 int angleOne;
// Integers to represent values from sticks and pots
//int ch1Value; int ch4Value; int ch5Value;
 int ch1Value; int ch3Value; int ch5Value; int ch6Value; 
// Boolean to represent switch value
//bool ch6Value;
 
// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  int angle = map(pwm,1000,2000,0,180);
  return angle;
}

// Left Gimble U/D
int readRudderGimble(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  return map(pwm,1000,2000,0,180);
}

// Right Gimble L/R
int readWinchGimble(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  int phi = floor(( 1500 - pwm ) / 25);
  return phi;
}

//SwC
int read3Switch(int channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  if (pwm < 1450) return 0;
  else if (pwm < 1950) return 1;
  else return 2;
}

// SwD
bool readSwitch(byte channelInput){
  int pwm = pulseIn(channelInput, HIGH);
  if (pwm < 1450) return 0;
  else return 1;
}


// Set rudder and winch servos
Servo rudder; Servo winch;
int rudderAngle; int winchAngle; int _mode = 0; int SwD = 0; int _base = 0;

void set_mode(int mode){
  _mode = mode;
}

void set_base(int swd){
  if(SwD == 0 and swd == 1){
    _base = random(100);
  }
  SwD = swd;
}

void servos_init(){ 
  // (do not need to set pinMode before, included in servo.attach())
  rudder.attach(PB0);
  winch.attach(PB1);
  setRudderAngle(90);
  setWinchAngle(90);
}

void setRudderAngle(int angle){
  rudderAngle = angle;
  rudder.write(angle);
}

void setWinchAngle(int angle){
  winchAngle = angle;
  winch.write(angle);
}

void setup(){
  // Set up serial monitor
  Serial.begin(115200);
  
  // Set all pins as inputs
  pinMode(CH1, INPUT); 
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);

  // Initialize servos
  servos_init(); 
  
}
 
 
void loop() {
  
  // Get values for each channel
  //ch1Value = readChannel(CH1, -100, 100, 0);
  //ch4Value = readChannel(CH4, -100, 100, 0);
  //ch5Value = readChannel(CH5, 0, 2, 1);
  //ch6Value = readSwitch(CH6, false);
  //ch1Value = readChannel(CH1);
  ch5Value = read3Switch(CH5);
  set_mode(ch5Value);
  ch6Value = readSwitch(CH6);
  set_base(ch6Value);
  ch3Value = readRudderGimble(CH3);
  ch1Value = readWinchGimble(CH1);
  if(_mode == 0){
    setRudderAngle(ch3Value);
    setWinchAngle(min(max(winchAngle + ch1Value,0),180));
  }

  //int rudder_phi = (180 / (long)(ch4Value - rudderAngle)) ;
  //int winch_phi = (180 / (long)(ch1Value - winchAngle));
  //setRudderAngle(max(min(rudderAngle + rudder_phi,180),0));
  //setWinchAngle(max(min(winchAngle + winch_phi,180),0));
  
  //ch1Value = pulseIn(CH1, HIGH);
  //angleOne = map(ch1Value,1000,2000,0,180);
  //ch4Value = pulseIn(CH4, HIGH);
  //ch5Value = pulseIn(CH5, HIGH);
  //ch6Value = pulseIn(CH6, HIGH);

   
  Serial.print(ch1Value);
  Serial.print(" | ");
  Serial.print(ch3Value);
  Serial.print(" | ");
  Serial.print(ch5Value);
  Serial.print(" | ");
  Serial.println(ch6Value);
  /*
  // Print to Serial Monitor
  //Serial.print(" | LEFT JOY: ");
  //Serial.print(ch2Value);
  Serial.print(" RUDDER ANGLE: ");
  Serial.print(rudderAngle);
  //Serial.print(" | RIGHT JOY: ");
  //Serial.print(ch1Value);
  Serial.print(" |  WINCH ANGLE: ");
  Serial.print(winchAngle);
  Serial.print(" | mode: ");
  Serial.print(_mode);
  Serial.print(" | base: ");
  Serial.println(_base);
  */
  delay(500);

  // practice with updating rudder and winch using 
}
