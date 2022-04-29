#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include "orientation.hpp"

//#include <hardIronCalibration.h>
// #include <Vector3D.h>
// For the fabs function in fixedWidthPrint
#include <Math.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accl = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified magn = Adafruit_LSM303_Mag_Unified(12345);

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

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Tilt Compensated Compass  Test"); 
  Serial.println("");

  /* Initialise the sensor */
  if(!accl.begin() || !magn.begin())  // also initalizes mag and accel sensors
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Enable auto-gain */
  magn.enableAutoRange(true);
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  
  // Boston geomagnetic field
  // Declination -14 degrees
  // Inclunation -6.4 degrees
  // Horizontal Intensity 20,000 nT
  // North Component 19,000 nT north
  // East Component 5,000 nT west
  // Vertical Component 48,000 nT down
  // Total Field 52,000 nT
  float declination = -3;

  // Boat  
  float hardiron_x = 0;
  float hardiron_y = 0;
  float hardiron_z = 0;
  //Vector3D hardiron;

  Orientation::setParameters(&accl, &magn, declination, hardiron_x, hardiron_y, hardiron_z);
  // Initalize hard iron values
  int n_iters = 5;
  Orientation::reCalibrate(n_iters);
}

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

void loop(void) 
{ 
  float roll;
  float pitch;
  float yaw;
  float heading;
  Orientation::reCalibrate(10);
  Orientation::calculate(roll, pitch, yaw, heading);
 
  Serial.print(" | roll:"); fixedWidthPrint(roll);
  Serial.print(" | pitch:"); fixedWidthPrint(pitch);  
  Serial.print(" | yaw:"); fixedWidthPrint(yaw);
  Serial.print(" | heading:"); fixedWidthPrint(heading);
  Serial.print(" | hardX:"); fixedWidthPrint(Orientation::hardiron_x);
  Serial.print(" | hardY:"); fixedWidthPrint(Orientation::hardiron_y);
  Serial.print(" | hardZ:"); fixedWidthPrint(Orientation::hardiron_z);
  Serial.print('\n');
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
