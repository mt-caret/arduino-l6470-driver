#include "Arduino.h"
#include "l6470.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

const int SPI_CS_LEFT = 8, SPI_CS_RIGHT = 9;
L6470 leftMotor(SPI_CS_LEFT), rightMotor(SPI_CS_RIGHT);

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  leftMotor.initialize();
  rightMotor.initialize();
  leftMotor.setStepMode(StepMode::microstep_128);
  rightMotor.setStepMode(StepMode::microstep_128);
  leftMotor.setKVal(KVal::hold, 255);
  leftMotor.setKVal(KVal::run, 255);
  leftMotor.setKVal(KVal::acc, 255);
  leftMotor.setKVal(KVal::dec, 255);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Sending Run command");
  leftMotor.run(Direction::forward, 4000);
  rightMotor.run(Direction::forward, 4000);

  //leftMotor.updateStatus();
  //leftMotor.printStatus();

  delay(10000);

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Sending SoftStop command");
  leftMotor.softStop();
  rightMotor.softStop();

  Serial.print("getParam result: ");
  Serial.print(leftMotor.getParam(REG_CONFIG), HEX);
  Serial.print('\n');

  //leftMotor.updateStatus();
  //leftMotor.printStatus();

  delay(1000);
  Serial.print("getStatus result: ");
  Serial.print(leftMotor.getStatus(), HEX);
  Serial.print('\n');
}
