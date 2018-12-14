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

uint8_t getLength(uint8_t param) {
  switch (param) {
    case REG_ABS_POS:    return REG_ABS_POS_LEN;
    case REG_EL_POS:     return REG_EL_POS_LEN;
    case REG_MARK:       return REG_MARK_LEN;
    case REG_SPEED:      return REG_SPEED_LEN;
    case REG_ACC:        return REG_ACC_LEN;
    case REG_DEC:        return REG_DEC_LEN;
    case REG_MAX_SPEED:  return REG_MAX_SPEED_LEN;
    case REG_MIN_SPEED:  return REG_MIN_SPEED_LEN;
    case REG_FS_SPD:     return REG_FS_SPD_LEN;
    case REG_KVAL_HOLD:  return REG_KVAL_HOLD_LEN;
    case REG_KVAL_RUN:   return REG_KVAL_RUN_LEN;
    case REG_KVAL_ACC:   return REG_KVAL_ACC_LEN;
    case REG_KVAL_DEC:   return REG_KVAL_DEC_LEN;
    case REG_INT_SPD:    return REG_INT_SPD_LEN;
    case REG_ST_SLP:     return REG_ST_SLP_LEN;
    case REG_FN_SLP_ACC: return REG_FN_SLP_ACC_LEN;
    case REG_FN_SLP_DEC: return REG_FN_SLP_DEC_LEN;
    case REG_K_THERM:    return REG_K_THERM_LEN;
    case REG_ADC_OUT:    return REG_ADC_OUT_LEN;
    case REG_OCD_TH:     return REG_OCD_TH_LEN;
    case REG_STALL_TH:   return REG_STALL_TH_LEN;
    case REG_STEP_MODE:  return REG_STEP_MODE_LEN;
    case REG_ALARM_EN:   return REG_ALARM_EN_LEN;
    case REG_CONFIG:     return REG_CONFIG_LEN;
    case REG_STATUS:     return REG_STATUS_LEN;
  }
  // TODO: assert false here.
  return 0;
}

void initialize(int chipSelectPin) {
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
