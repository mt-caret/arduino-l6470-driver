#include "Arduino.h"
#include "SPI.h"
#include "l6470.h"

/*
https://github.com/electricimp/reference/tree/master/hardware/L6470
starting motor:
1. set microstepping mode
2. configure oscillator source and PWM multiplier
3. read the status register to clear overcurrent bit

other initialization tweaks:
1. set max speed / full-stepping speed
2. set maximum acceleration/deceleration speeds
3. set k-values
*/

void initialize_spi(int chipSelectPin) {
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setDataMode(SPI_MODE3);
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);

    resetDevice(chipSelectPin);
}

uint8_t transferByte(int chipSelectPin, uint8_t data) {
    digitalWrite(chipSelectPin, LOW);
    uint8_t output = SPI.transfer(data);
    digitalWrite(chipSelectPin, HIGH);
    return output;
}

uint16_t transferTwoBytes(int chipSelectPin, uint16_t data) {
  uint16_t outputMSB = transferByte(chipSelectPin, data >> 8) << 8;
  uint16_t outputLSB = transferByte(chipSelectPin, data);
  uint16_t result = outputMSB | outputLSB;
  Serial.print(result, HEX);
  Serial.print('\n');
  return result;
}

uint32_t sendBytes(int chipSelectPin, uint32_t value, uint8_t length) {
    uint32_t result = 0;
    while (true) {
        uint8_t shift_amount = (length / 8) * 8;
        result |= transferByte(chipSelectPin, (uint8_t) (value >> shift_amount));
        if (shift_amount == 0) break;
        result <<= 8;
        length -= 8;
    }
    Serial.print(result, HEX);
    Serial.print('\n');
    return result;
}

void setParam(int chipSelectPin, uint8_t param, uint32_t value, uint8_t length) {
    transferTwoBytes(chipSelectPin, param & 0b00011111);
    sendBytes(chipSelectPin, value, length);
}

uint32_t getParam(int chipSelectPin, uint8_t param, uint8_t length) {
    transferTwoBytes(chipSelectPin, (param & 0b00011111) | 0b00100000);
    uint32_t ret = 0;
    while (true) {
        ret |= transferByte(chipSelectPin, 0x00);
        if (length <= 8) break;
        ret = ret << 8;
        length -= 8;
    }
    return ret;
}

void run(int chipSelectPin, bool forward, uint32_t speed) {
    transferTwoBytes(chipSelectPin, 0b01010000 | (forward ? 1 : 0));
    sendBytes(chipSelectPin, speed & 0xffffff, 22);
}

void softStop(int chipSelectPin) {
    transferTwoBytes(chipSelectPin, 0b10110000);
}

void hardStop(int chipSelectPin) {
    transferTwoBytes(chipSelectPin, 0b10111000);
}

void resetDevice(int chipSelectPin) {
    transferTwoBytes(chipSelectPin, 0b11000000);
}

//void softHiZ(void) {
//    SPI.transfer(0b10100000);
//}
//
//void softHiZ(void) {
//    SPI.transfer(0b10101000);
//}
