/* Frequency is 60 Hz*/
float freq,period;
int ontime,offtime,duty;
unsigned long time_on;
unsigned long time_off;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(PA2,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
   //FrequencyReader();        // Call Frequency Reader Subroutine
                            // Connect pin you want to read frequency of to analog pin 0    
   //ontime = analogRead(PA2);
   //Serial.println(ontime);
   noInterrupts();
   time_on = pulseIn(PA2, HIGH,50000);
   time_off = pulseIn(PA2, LOW,50000);
   interrupts();
   unsigned long angle = ((time_on * 1026) / (time_on + time_off)) -1;
   unsigned long pos = 0;
   if (angle <= 1022) {
    pos = angle;
   }
   else if ( angle == 1024){
    pos = 1023;
   }
   Serial.println(time_on);
   Serial.println(time_off);
   Serial.println(angle);
   Serial.println(pos);
   
   delay(2000);
   
   /*
   offtime = pulseIn(PA0,LOW);
   period = ontime+offtime;
   freq = 1000000.0/period;
   duty = (ontime/period)*100;     
   Serial.print("Frequency: ");                 // Display frequency in Serial Monitor
  Serial.print(freq);
 Serial.println(" Hz");
 Serial.print("duty: ");                 // Display frequency in Serial Monitor
 Serial.print(duty);
 Serial.println(" Hz");   
 */            
}
//
void FrequencyReader()
{
 unsigned long highPulse = pulseIn(A0,HIGH);  // Read the duration that the square wave is high
 unsigned long lowPulse = pulseIn(A0,LOW);    // Read the duration that the square wave is low
 float period = highPulse + lowPulse;         // Add both of these durations and it will give you the period of the square wave in microseconds
 float frequency = 1 / (period/1000000);      // f=1/T ; We divide the period by 1,000,000 to convert to seconds
 Serial.print("Frequency: ");                 // Display frequency in Serial Monitor
 Serial.print(frequency);
 Serial.println(" Hz");
 delay(500);
}
