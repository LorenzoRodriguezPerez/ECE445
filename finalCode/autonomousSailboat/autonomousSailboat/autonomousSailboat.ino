#include <Servo.h>
#include <Wire.h>

#include <TinyGPS++.h>
#include <LSM303.h>
#include <HardwareSerial.h>
#include <RotaryEncoder.h>
#include <Math.h>

/* TELEMETRY INFO */
HardwareSerial teleSerial(PB10);
unsigned long prevTimeTele;
const int telePeriod = 1000;

/* DEBUG -> UNPLUG CHANNELS 5 AND 6, CONNECT CHANNEL 5 TO RX OF CONVERTER */
//HardwareSerial debugSerial(PA3,PA2);

/* GPS VARIABLES */
TinyGPSPlus gps;
HardwareSerial gpsSerial(PA10, PA9);
static double _lat,_lng, base_lat, base_lng;
static float _course, _knots, _mph;

/* PID VARIABLES */
static float desiredHeading = 0; 
const float PID_period = 1; //in ms
float PID_prevIntegral;
const long Kp = 3.1;
const float Td = 0.1;
const float Ti = 3;
const float Ki = Kp / Ti;
const float Kd = Kp * Td;
//float prevDeviation[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* CALIBRATION OF ECOMPASS */
unsigned long calibrationTime = 15000; //in ms

/* Setup pointers to eCompass module PB6 (SCL), PB7 (SDA) -> DONE IN WIRE.BEGIN() */
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767,32767,32767}, running_max = {-32768,-32768,-32768};

/* RECEIVER CHANNEL ASSIGNMENTS 
 *  J7 = PA0, J8 PA1, J9 PA2, J10 PA3, J11 PA6
 *  CHANNEL 1: RIGHT JOY L/R -> Winch Control
 *  CHANNEL 2: RIGHT JOY U/D -> N/A
 *  CHANNEL 3: LEFT JOY U/D -> Rudder Control
 *  CHANNEL 4: LEFT JOY L/R -> N/A
 *  CHANNEL 5: SWC -> 1: Manual, 2: Autonomous, 3: Return to Base
 *  CHANNEL 6: SWD -> 0->1: Set New Base Position
*/

#define CH1 PA0
#define CH3 PA1
#define CH5 PA2
#define CH6 PA3
//#define DEBUG PA6

// J2 PB14, J3 PB15
#define WINCH_SERVO PB14
#define RUDDER_SERVO PB15

/* ROTARY ENCODER SETUP */
#define WINDVANE_W PB13 
#define WINDVANE_G PA11
RotaryEncoder *encoder = nullptr;




// Rotary Encoder ISR
void checkPosition()
{
  encoder->tick();
}

// Retrieving wind direction from rotary encoder "tick" position
int getWindDir(){
  int pos = abs((int)encoder->getPosition()) % 1190; //1190 full rotation
  int angle;
  if ((int)encoder->getDirection() <0) {
    angle = 360 - map(pos,0,1190,0,360);
    }
  else {angle = map(pos,0,1190,0,360);}
  return angle;
}

/* SERVO \ RECEIVER SETUP */
Servo winchServo, rudderServo; 
volatile int val[4]; volatile long start[4]; volatile long pulses[4]; volatile long pulseWidth[4];

/* CALIBRATED RANGE OF PWM READS FOR EACH CHANNEL  */
// CH1,CH3,CH5,CH6
int range[4][2] = {{950,2050},{1000,2000},{950,2010},{900,2100}};
// RUDDER,WINCH
int servoRange[2][2] = {{37,165},{72,110}};

void ISR_WINCH()  { PulseTimer(0);}
void ISR_RUDDER() { PulseTimer(1);}
void ISR_MODE()   { PulseTimer(2);}
void ISR_BASE()   { PulseTimer(3);}

/* TEMP VALS, CONTROL LIB TO BE ADDED */
static float _winchAngle, _rudderAngle, _mode, _base;
static int phi;
volatile float newPhi;
volatile float _dev = 85;
void (*setVal[4])() = { setWinch, setRudder, setModeISR,setBase};
//PID autonomousPID;

/* WIND VANE */
static float windAngle;
/* eCompass */
static float roll, _heading;
/* PID */
static float distanceTo, period,prevTime;


unsigned long tprev_manual, tprev_autonomous, tprevreturntobase = 0;
/* RUDDER IS CALIBRATED TO EVEN RANGE W RSPT TO BOAT (36,165) */
void setRudder(){
  if (_mode == 0){
    float newAngle = map(val[1],range[1][0],range[1][1],servoRange[0][0],servoRange[0][1]);
    if (newAngle < (_rudderAngle - 7.7) || newAngle > (_rudderAngle + 7.7)  ){
      _rudderAngle = newAngle;
     }
  }
}

/* VALUE GOES TO MAX WHEN REMOTE TURNS OFF -> FIX */
void setWinch(){
  /* Map PWM into range: -3,-2,-1,0,1,2,3 (multiplier for adjusting tightness) */
  newPhi = map(val[0],range[0][0],range[0][1],-3,4);
  float newAngle;
  if (newPhi > 0){newAngle = constrain(_winchAngle + ( (newPhi*newPhi) / 100),servoRange[1][0],servoRange[1][1]);}
  else {newAngle = constrain(_winchAngle -( (newPhi*newPhi) / 100),servoRange[1][0],servoRange[1][1]);}
  _winchAngle = newAngle;
}

void setModeISR(){
  /* MANUAL */
  if(val[2] <1400){
    _mode = 0;
  }
  /* RETURN TO BASE */
  else if (val[2] > 1600){
    _mode = 2;
  }
  /* AUTONOMOUS */
  else{
    _mode = 1;
  }
}

void setBase(){
  /* USER CAN ONLY SET BASE IN MANUAL OR AUTONOMOUS MODE */
  if (_mode != 2){
    if(val[3] < 1500){ _base = 0; }
    else { if(_base ==0) { base_lat = _lat; base_lng = _lng; }
       _base = 1;
       }  
    }
}

void PulseTimer(int ISN){
  pulses[ISN] = micros();
  if (pulses[ISN] > start[ISN]){
    pulseWidth[ISN] = pulses[ISN] - start[ISN];
    start[ISN] = pulses[ISN];
  }
  
  if (range[ISN][0] < pulseWidth[ISN] && pulseWidth[ISN] < range[ISN][1]){
    val[ISN] = pulseWidth[ISN];
    setVal[ISN]();
  }
}


double getRudderAngle(float deviation, float period){
    //float derivative = (deviation - prevDeviation) / period;
    /*
    float derivative = (deviation + (13*prevDeviation[0]) + (77*prevDeviation[1]) + (273*prevDeviation[2]) + (637*prevDeviation[3]) + (1001*prevDeviation[4]) + (1001*prevDeviation[5]) 
        + (429*prevDeviation[6]) - (429* prevDeviation[7]) - (1001* prevDeviation[8]) - (1001* prevDeviation[9]) - (637* prevDeviation[10]) - (273* prevDeviation[11]) - (77* prevDeviation[12]) - (13* prevDeviation[13])  - prevDeviation[14]) / (16384 * period);
    */
    // PI CONTROLLER
    float integral  = PID_prevIntegral + (deviation * period);
    // PID CONTROLLER
    //double newAngle = (Kp * deviation) + (Kd * derivative) + (Ki * integral);
    
    double newAngle = Kp*deviation + (Ki * integral);
    
    // Integral WindUp
    if (newAngle >= 45 || newAngle <= - 45 ){ integral  = PID_prevIntegral; }

    PID_prevIntegral = integral;
    
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
    
}


void setup() {
  /* TELEMETRY SETUP */
  teleSerial.begin(57600);
  
  /* Winch and Rudder Servo Setup */
  winchServo.attach(WINCH_SERVO);
  rudderServo.attach(RUDDER_SERVO);
  
  _winchAngle = 90; _rudderAngle = 45;
  winchServo.write(_winchAngle);
  rudderServo.write(_rudderAngle);

  /* eCOMPASS setup*/
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  
  /* PID */
  PID_prevIntegral = 0;
  
  /* GPS setup */
  gpsSerial.begin(9600);
  // Set Base Position
  base_lat = 0.000000; base_lng = 0.000000;
  if (gpsSerial.available()){
      gps.encode(gpsSerial.read());
      base_lat = gps.location.lat();
      base_lng = gps.location.lng();
  }
  
  /* RECEIVER SETUP */
  _mode = 0;
  pinMode(CH1, INPUT_PULLUP);
  pinMode(CH3, INPUT_PULLUP);
  pinMode(CH5, INPUT_PULLUP);
  pinMode(CH6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CH1),ISR_WINCH,CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3),ISR_RUDDER,CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH5),ISR_MODE,CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH6),ISR_BASE,CHANGE);

  /* ROTARY ENCODER SETUP */
  // WHITE = PB13, GREEN = PB12
  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(WINDVANE_W, WINDVANE_G, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(WINDVANE_W), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WINDVANE_G), checkPosition, CHANGE);
  
}

void loop() {
  // Update eCompass
  compass.read();
  _heading = compass.heading();
  roll = atan2(compass.a.y,compass.a.z) * 57.2957;
  
  // Update Shaft Encoder
  windAngle = getWindDir();
  
  // Update GPS
  if (gpsSerial.available()){ 
    gps.encode(gpsSerial.read()); 
    _lat = gps.location.lat();
    _lng = gps.location.lng();
    if(base_lat == 0 || base_lng == 0){ base_lat = _lat; base_lng = _lng; }
      _knots = gps.speed.knots();
      _mph = gps.speed.mph();
      if (gps.course.isValid()){_course = gps.course.deg();}
      distanceTo = gps.distanceBetween(_lat,_lng,base_lat,base_lng);
  }
  
 
  /* MANUAL MODE */
  
  if (_mode == 0) {
    // Set winch and rudder servos
    winchServo.write(_winchAngle);
    rudderServo.write(_rudderAngle);

    unsigned long telemetry_manual = millis();
    if (telemetry_manual >= tprev_manual + 1000) {
        //teleSerial.print(_base);
        teleSerial.print("MANUAL ");
        teleSerial.print(" lati ");
        //teleSerial.print(_lat);
        printDouble(_lat,1000000);
        teleSerial.print(" lngi ");
        //teleSerial.print(_lng);
        printDouble(_lng,1000000);
        teleSerial.print(" windAngle ");
        teleSerial.print(windAngle);
        teleSerial.print(" winchAngle ");
        teleSerial.print(_winchAngle);
        teleSerial.print(" rudderAngle ");
        teleSerial.print(_rudderAngle);
        teleSerial.print(" desiredHeading ");
        teleSerial.print(desiredHeading);
        teleSerial.print(" heading ");
        teleSerial.print(_heading);
        teleSerial.print(" course ");
        teleSerial.print(_course);
        teleSerial.print(" roll ");
        teleSerial.print(roll);
        teleSerial.print(" knots ");
        teleSerial.print(_knots);
        teleSerial.print(" distanceTobase ");
        teleSerial.print(distanceTo);
        teleSerial.print(" baselat ");
        //teleSerial.print(base_lat);
        printDouble(base_lat,1000000);
        teleSerial.print(" baselng ");
        //teleSerial.println(base_lng);
        printDoublenewL(base_lng,1000000);
        tprev_manual = telemetry_manual;
        
       }
   }
   
  /* AUTONOMOUS MODE */
  else if(_mode == 1) {
    
    // Set Desired Heading to Current Heading 
    desiredHeading = _heading; 
   
    // Init new instance of PID (initialize derivative values to 0) 
    PID_prevIntegral = 0;
    prevTime = 0;

    // Setup adjustment for heeling angle
    float heelingPhi = 0;
    
    // AUTONOMOUS MODE LOOP
    while (_mode == 1) {
      // Update eCompass
      compass.read();
      _heading = compass.heading();
      roll = atan2(compass.a.y,compass.a.z) * 57.2957;
      
      // Update Shaft Encoder
      windAngle = getWindDir();
  
      // Update GPS
      if (gpsSerial.available()){ 
        gps.encode(gpsSerial.read()); 
        _lat = gps.location.lat();
        _lng = gps.location.lng();
        if(base_lat == 0 || base_lng == 0){ base_lat = _lat; base_lng = _lng; }
          _knots = gps.speed.knots();
          _mph = gps.speed.mph();
          _course = gps.course.deg();
          //if (gps.course.isValid()){_course = gps.course.deg();}
          //desiredHeading = gps.courseTo(_lat,_lng,base_lat, base_lng);  
          distanceTo = gps.distanceBetween(_lat,_lng,base_lat,base_lng);
      }
      
      float deviation = desiredHeading - _heading;
      //358 - 14
      if (deviation <= -180) {deviation = 360 + deviation;}
      else if (deviation >= 180) {deviation = -(360 - deviation);}
      
      unsigned long currentTime = millis();
      double newRudderAngle;
      // Output from PID is RELATIVE rudder angle between -45, 45
      if (currentTime - prevTime >= PID_period) {
        newRudderAngle = constrain (getRudderAngle(deviation, PID_period / 1000), -45,45);
        prevTime = currentTime;
      }
      _rudderAngle = map((int)newRudderAngle, -45, 45, 40,165);
      
      // Set Winch According to Relative Wind Direction
      if     (windAngle >=0 && windAngle <= 20  ) { _winchAngle = 72; }
      else if(windAngle >20 && windAngle <= 60  ) { _winchAngle = 72; }
      else if(windAngle >60 && windAngle <= 70  ) { _winchAngle = 72; }
      else if(windAngle >70 && windAngle <= 110 ) { _winchAngle = 75; }
      else if(windAngle >110 && windAngle <= 140) { _winchAngle = 80; }
      else if(windAngle >140 && windAngle <= 220) { _winchAngle = 110;}
      else if(windAngle >220 && windAngle <= 250) { _winchAngle = 80; }
      else if(windAngle >250 && windAngle <= 290) { _winchAngle = 75; }
      else if(windAngle >290 && windAngle <= 300) { _winchAngle = 72; }
      else if(windAngle >300 && windAngle <= 340) { _winchAngle = 72; }
      else                                        { _winchAngle = 72; }

      // Adjust winch angle for heeling angle
      //115 is completely tipped over.. 
      // max heeling phi = 20
      if ((abs(roll) < 160) && (abs(roll) > 115)){ 
        if (heelingPhi >= (servoRange[1][1] - servoRange[1][0])){ heelingPhi = (servoRange[1][1] - servoRange[1][0]);}
        else {heelingPhi = heelingPhi + 0.02;}}
      else { 
        if (heelingPhi <= 0.08) { heelingPhi = 0; }
        else { heelingPhi = heelingPhi - 0.1;}}
      int tempWinch = _winchAngle + heelingPhi;
      _winchAngle = min(servoRange[1][1],tempWinch);
      
      // Set servos
      rudderServo.write(_rudderAngle);
      winchServo.write(_winchAngle);

      unsigned long telemetry_autonomous = millis();
      if (telemetry_autonomous >= tprev_autonomous + 1000) {
        
        teleSerial.print("AUTONOMOUS ");
        teleSerial.print(" lati ");
        //teleSerial.print(_lat);
        printDouble(_lat,1000000);
        teleSerial.print(" lngi ");
        //teleSerial.print(_lng);
        printDouble(_lng,1000000);
        teleSerial.print(" windAngle ");
        teleSerial.print(windAngle);
        teleSerial.print(" winchAngle ");
        teleSerial.print(_winchAngle);
        teleSerial.print(" rudderAngle ");
        teleSerial.print(_rudderAngle);
        teleSerial.print(" desiredHeading ");
        teleSerial.print(desiredHeading);
        teleSerial.print(" heading ");
        teleSerial.print(_heading);
        teleSerial.print(" course ");
        teleSerial.print(_course);
        teleSerial.print(" roll ");
        teleSerial.print(roll);
        teleSerial.print(" knots ");
        teleSerial.print(_knots);
        teleSerial.print(" distanceTobase ");
        teleSerial.print(distanceTo);
        teleSerial.print(" baselat ");
        //teleSerial.print(base_lat);
        printDouble(base_lat,1000000);
        teleSerial.print(" baselng ");
        printDoublenewL(base_lng,1000000);
        //teleSerial.println(base_lng);
        
        tprev_autonomous = telemetry_autonomous;
       }
    }
   }
   
   /* RETURN TO BASE MODE */
   else if(_mode == 2) {
    // Only works if GPS connection is established
    if (_lat == 0 || _lng == 0) { teleSerial.println("Return_to_Base_not_available._No_GPS_connection."); }
    else {
      // Init new instance of PID (initialize derivative values to 0) 
      PID_prevIntegral = 0;
      prevTime = 0;

      // Setup adjustment for heeling angle
      float heelingPhi = 0;
      if (gpsSerial.available()){
        desiredHeading = 101;//gps.courseTo(base_lat,base_lng,_lat,_lng);
      }
       
      while (_mode == 2){
        // Update eCompass
        compass.read();
        _heading = compass.heading();
        roll = atan2(compass.a.y,compass.a.z) * 57.2957;
      
        // Update Shaft Encoder
        windAngle = getWindDir();
  
        // Update GPS
       if (gpsSerial.available()){ 
          gps.encode(gpsSerial.read()); 
          _lat = gps.location.lat();
          _lng = gps.location.lng();
          if(base_lat == 0 || base_lng == 0){ base_lat = _lat; base_lng = _lng; }
          _knots = gps.speed.knots();
          _mph = gps.speed.mph();
          //if (gps.course.isValid()){_course = gps.course.deg();}
          _course = gps.course.deg();
          distanceTo = gps.distanceBetween(_lat,_lng,base_lat,base_lng);
       }
      
      float deviation = desiredHeading - _heading;
      if (deviation <= -180) {deviation = 360 + deviation;}
      else if (deviation >= 180) {deviation = -(360 - deviation);}
      
      unsigned long currentTime = millis();
      double newRudderAngle;
      // Output from PID is RELATIVE rudder angle between -45, 45
      if (currentTime - prevTime >= PID_period) {
        newRudderAngle = constrain (getRudderAngle(deviation, PID_period / 1000), -45,45);
        prevTime = currentTime;
      }
      _rudderAngle = map((int)newRudderAngle, -45, 45, 40,165);
      
      // Set Winch According to Relative Wind Direction
      if     (windAngle >=0 && windAngle <= 20  ) { _winchAngle = 72; }
      else if(windAngle >20 && windAngle <= 60  ) { _winchAngle = 72; }
      else if(windAngle >60 && windAngle <= 70  ) { _winchAngle = 72; }
      else if(windAngle >70 && windAngle <= 110 ) { _winchAngle = 75; }
      else if(windAngle >110 && windAngle <= 140) { _winchAngle = 80; }
      else if(windAngle >140 && windAngle <= 220) { _winchAngle = 110;}
      else if(windAngle >220 && windAngle <= 250) { _winchAngle = 80; }
      else if(windAngle >250 && windAngle <= 290) { _winchAngle = 75; }
      else if(windAngle >290 && windAngle <= 300) { _winchAngle = 72; }
      else if(windAngle >300 && windAngle <= 340) { _winchAngle = 72; }
      else                                        { _winchAngle = 72; }

      // Adjust winch angle for heeling angle
      //115 is completely tipped over.. 
      // max heeling phi = 20
      if ((abs(roll) < 160) && (abs(roll) > 115)){ 
        if (heelingPhi >= (servoRange[1][1] - servoRange[1][0])){ heelingPhi = (servoRange[1][1] - servoRange[1][0]);}
        else {heelingPhi = heelingPhi + 0.02;}}
      else { 
        if (heelingPhi <= 0.08) { heelingPhi = 0; }
        else { heelingPhi = heelingPhi - 0.1;}}
      int tempWinch = _winchAngle + heelingPhi;
      _winchAngle = min(servoRange[1][1],tempWinch);
      
      // Set servos
      rudderServo.write(_rudderAngle);
      winchServo.write(_winchAngle);

      unsigned long telemetry_return = millis();
      if (telemetry_return >= tprevreturntobase + 1000) {
        teleSerial.print("RETURN_TO_BASE ");
        teleSerial.print(" lati ");
        printDouble(_lat,1000000);
        //teleSerial.print(_lat);
        teleSerial.print(" lngi ");
        //teleSerial.print(_lng);
        printDouble(_lng,1000000);
        teleSerial.print(" windAngle ");
        teleSerial.print(windAngle);
        teleSerial.print(" winchAngle ");
        teleSerial.print(_winchAngle);
        teleSerial.print(" rudderAngle ");
        teleSerial.print(_rudderAngle);
        teleSerial.print(" desiredHeading ");
        teleSerial.print(desiredHeading);
        teleSerial.print(" heading ");
        teleSerial.print(_heading);
        teleSerial.print(" course ");
        teleSerial.print(_course);
        teleSerial.print(" roll ");
        teleSerial.print(roll);
        teleSerial.print(" knots ");
        teleSerial.print(_knots);
        teleSerial.print(" distanceTobase ");
        teleSerial.print(distanceTo);
        teleSerial.print(" baselat ");
        //teleSerial.print(base_lat);
        printDouble(base_lat,1000000);
        teleSerial.print(" baselng ");
        //teleSerial.println(base_lng);
        printDoublenewL(base_lng,1000000);
        
        tprevreturntobase = telemetry_return;
       }
    } // end while loop return to base
    } // end else
   } // end mode 2
   
}

void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

    teleSerial.print (int(val));  //prints the int part
    teleSerial.print("."); // print the decimal point
    unsigned int frac;
    if(val >= 0)
        frac = (val - int(val)) * precision;
    else
        frac = (int(val)- val ) * precision;
    teleSerial.print(frac,DEC) ;
} 

void printDoublenewL( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

    teleSerial.print (int(val));  //prints the int part
    teleSerial.print("."); // print the decimal point
    unsigned int frac;
    if(val >= 0)
        frac = (val - int(val)) * precision;
    else
        frac = (int(val)- val ) * precision;
    teleSerial.println(frac,DEC) ;
} 

/* NEED TO USE INTERNAL CLOCK INSTEAD OF CRYSTAL */
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){ while(1); }
  /* Initializes the CPU, AHB and APB buses clocks*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){ while(1); }
}

/* For Debugging */
/*
void fixedWidthPrint(float value){
  // Bug: one too many spaces when value === 10
  int characters = 0;
   
  // Count the number of digits before the decimal point
  for(int i = 10000; i > 0; i /= 10){
     if((int)fabs(value) >= i){
       characters ++;
     }
  } 
  
  if(characters == 0){
    characters = 1; // Minimum of 1 character if we print '0'
  }
  
  if(fabs(value) != value){ // Is it negative?
    characters++;
  }
  
  for(int i = 6; i > characters; i--){
    Serial.print(' ');
  }
  Serial.print(value, 2);
}


void debug_receiver() {
  
  Serial.print("  CH1 | ");
  Serial.print(_winchAngle);
  //Serial.print(phi);
  Serial.print("  CH3 | ");
  //Serial.println(val[1]);
  Serial.print(_rudderAngle);
  
  Serial.print("   SwC: ");
  //Serial.print(val[2]);
  Serial.print(_mode);
  Serial.print("  SwD: ");
  Serial.println(val[1]);
  Serial.println(_base);
  
  }
  
void debug_eCompass(){
  
  Serial.print("  roll: ");
  Serial.print(roll);
  Serial.print("  pitch: ");
  Serial.print(pitch);
  Serial.print("  yaw: ");
  Serial.print(yaw);
  Serial.print("  heading: ");
  Serial.println(heading);
  */
  /*
  Serial.print(" | roll:"); fixedWidthPrint(roll);
  Serial.print(" | pitch:"); fixedWidthPrint(pitch);  
  Serial.print(" | yaw:"); fixedWidthPrint(yaw);
  Serial.print(" | heading:"); fixedWidthPrint(heading);
  Serial.print(" | hardX:"); fixedWidthPrint(Orientation::hardiron_x);
  Serial.print(" | hardY:"); fixedWidthPrint(Orientation::hardiron_y);
  Serial.print(" | hardZ:"); fixedWidthPrint(Orientation::hardiron_z);
  Serial.print('\n');
  
  }
void GPS_print_debug(void) {
    teleSerial.print(_lat);
    teleSerial.print(" | longitude:");
    teleSerial.print(_lng);
    teleSerial.print(" | knots:");
    teleSerial.print(_knots);
    teleSerial.print(" | course: ");
    teleSerial.print(_course);
    teleSerial.print(" | base lat:");
    teleSerial.print(base_lat);
    teleSerial.print(" | base lng: ");
    teleSerial.print(base_lng);
    teleSerial.print(" | course to base: ");
    teleSerial.print(desiredHeading);
    teleSerial.print(" | distance to base: ");
    teleSerial.println(distanceTo); 
}

void control_print_debug(void){
    teleSerial.print("mode: ");
    teleSerial.print(_mode);
    teleSerial.print(" | servoRudderAngle: ");
    teleSerial.print(_rudderAngle);
    teleSerial.print(" | winchRudderAngle: ");
    teleSerial.print(_winchAngle);
    teleSerial.print(" | rudderAngle: ");
    teleSerial.print(_actualRudderAngle);
    teleSerial.print(" | winchAngle: ");
    teleSerial.print(_actualWinchAngle);
    teleSerial.print(" | windDirection: ");
    teleSerial.println(windAngle);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  accl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); 
  Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); 
  Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); 
  Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); 
  Serial.print(sensor.max_value); 
  Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); 
  Serial.print(sensor.min_value); 
  Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
*/
/* void autonomous_print_user(void){
    teleSerial.println("Autonomous Mode Activated");
    teleSerial.print("Boat currently located at lat: ");
    teleSerial.print(_lat);
    teleSerial.print(" , lon: ");
    teleSerial.print(_lng);
    teleSerial.print(". Maintaining a heading of (eCompass): ");
    teleSerial.print(heading);
    teleSerial.print(" , (GPS): ");
    teleSerial.println(_course);
 } */
 
// RUNNING MIN/MAX ECOMPASS
/*
  running_min.x = min(running_min.x,compass.m.x);
  running_min.y = min(running_min.y,compass.m.y);
  running_min.z = min(running_min.z,compass.m.z);
  running_max.x = max(running_max.x,compass.m.x);
  running_max.y = max(running_max.y,compass.m.y);
  running_max.z = max(running_max.z,compass.m.z);
  compass.m_min = (LSM303::vector<int16_t>){running_min.x,running_min.y,running_min.z};
  compass.m_max = (LSM303::vector<int16_t>){running_max.x,running_max.y,running_max.z};
  
  _heading = compass.heading();
  roll = atan2(compass.a.y,compass.a.z) * 57.2957;
  */


  // Initialize the sensor -> OLD ECOMPASS
  /*
  if(!accl.begin() || !magn.begin()){ 
    //teleSerial.println("eCompass not connected");
   // while(1);
   }
  else {
    // Enable auto-gain 
    magn.enableAutoRange(true);
    //accel.setRange(LSM303_RANGE_4G);
    //accel.setMode(LSM303_MODE_NORMAL);
    
    // Declination in Champaign, IL
    float declination = -3;
  
    // TO DO RECALIBRATE HARD IRON VALUES
    float hardiron_x = -14;
    float hardiron_y = -17;
    float hardiron_z = -17;
    
    Orientation::setParameters(&accl, &magn, declination, hardiron_x, hardiron_y, hardiron_z);
    //teleSerial.println("eCompass initialization successful");
    
  }
  */
