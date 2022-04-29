#include <IBusBM.h>
#include <Servo.h>

IBusBM IBus;    // IBus object
//Servo myservo1;  // create servo object to control a servo
//Servo myservo2;  // create servo object to control a servo
// only need to connect to PA3 (rx)








HardwareSerial ser(PA3,PA2);
void setup() {
  Serial.begin(115200);
  IBus.begin(ser);    // iBUS object connected to serial0 RX pin
  //myservo1.attach(8);     // attaches the servo on pin 8 to the servo1 object
  //myservo2.attach(9);     // attaches the servo on pin 9 to the servo2 object
}

void loop() {
  int val;
  int i = 0;
  while (i < 6) {
    val = IBus.readChannel(i);
    if (val != 0){
      Serial.println("Channel: ");
      Serial.println(i+1);
      Serial.println(" = ");
      Serial.println(val);
      Serial.println(" \n");
    }
    delay(200);
  }
}
