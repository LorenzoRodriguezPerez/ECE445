// Test sketch

#define BUT_PIN PIN_PB4
#define LED_PIN PIN_PB2
#define POT_PIN A3
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUT_PIN, INPUT);
  // enable internal pullup
  digitalWrite(BUT_PIN, HIGH);
}

void loop() {
  // read potentiometer and rescale to 0-1s delay
  int delay_time = 1000 * int32_t(analogRead(POT_PIN)) / 1024;

  // blink while button depressed
  if (digitalRead(BUT_PIN) == LOW) {
    digitalWrite(LED_PIN, HIGH);
    delay(delay_time);
    digitalWrite(LED_PIN, LOW);
    delay(delay_time);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}
