#include "Arduino.h"
#include "l6470.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

const int SPI_CS = 8;
L6470 l6470(SPI_CS);

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  l6470.initialize();
  l6470.setStepMode(StepMode::microstep_128);
  //l6470.setKVal(KVal::hold, 100);
  //l6470.setKVal(KVal::run, 100);
  //l6470.setKVal(KVal::acc, 100);
  //l6470.setKVal(KVal::dec, 100);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Sending Run command");
  l6470.run(Direction::forward, 3000);
  delay(2000);

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Sending SoftStop command");
  l6470.softStop();
  delay(2000);

  Serial.print(l6470.getParam(REG_CONFIG), HEX);
  Serial.print('\n');
}
