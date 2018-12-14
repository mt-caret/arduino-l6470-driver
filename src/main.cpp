#include "Arduino.h"
#include "l6470.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

const int SPI_CS = 8;

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    initialize_spi(SPI_CS);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Sending Run command");
    run(SPI_CS, true, 1000);
    delay(5000);

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Sending SoftStop command");
    softStop(SPI_CS);
    //Serial.println("Setting led to low");
    delay(5000);

    Serial.print(getParam(SPI_CS, REG_CONFIG, REG_CONFIG_LEN), HEX);
    Serial.print('\n');
}
