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

L6470::L6470(int chipSelectPin)
  : chipSelectPin(chipSelectPin)
  , spiSettings (SPISettings(5000000, MSBFIRST, SPI_MODE3)) {
}

uint8_t L6470::getLength(uint8_t param) {
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

void L6470::initialize() {
  SPI.begin();
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);

  resetDevice();
}

uint8_t L6470::transferByte(uint8_t data) {
  digitalWrite(chipSelectPin, LOW);
  uint8_t output = SPI.transfer(data);
  digitalWrite(chipSelectPin, HIGH);
  Serial.print(data, HEX);
  Serial.print(" -> ");
  Serial.print(output, HEX);
  Serial.print('\n');
  return output;
}

uint32_t L6470::sendBytes(uint32_t value, uint8_t length) {
  uint32_t result = 0;
  while (true) {
    uint8_t shift_amount = (length / 8) * 8;
    result |= transferByte((uint8_t) (value >> shift_amount));
    if (shift_amount == 0) break;
    result <<= 8;
    length -= 8;
  }
  return result;
}

void L6470::setParam(uint8_t param, uint32_t value, uint8_t length) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_SET_PARAM | (param & 0b00011111));
  sendBytes(value, length);
  SPI.endTransaction();
}

uint32_t L6470::getParam(uint8_t param, uint8_t length) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_GET_PARAM | (param & 0b00011111));
  uint32_t ret = 0;
  while (true) {
    ret |= transferByte(0x00);
    if (length <= 8) break;
    ret = ret << 8;
    length -= 8;
  }
  SPI.endTransaction();
  return ret;
}

void L6470::run(Direction direction, uint32_t speed) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_RUN | static_cast<uint8_t>(direction));
  sendBytes(speed, 22);
  SPI.endTransaction();
}

void L6470::stepClock(Direction direction) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_STEP_CLOCK | static_cast<uint8_t>(direction));
  SPI.endTransaction();
}

void L6470::move(Direction direction, uint32_t steps) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_MOVE | static_cast<uint8_t>(direction));
  sendBytes(steps, 22);
  SPI.endTransaction();
}

void L6470::softStop(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_SOFT_STOP);
  SPI.endTransaction();
}

void L6470::hardStop(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_HARD_STOP);
  SPI.endTransaction();
}

void L6470::softHiZ(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_SOFT_HI_Z);
  SPI.endTransaction();
}

void L6470::hardHiZ(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_HARD_HI_Z);
  SPI.endTransaction();
}

void L6470::resetPos(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_RESET_POS);
  SPI.endTransaction();
}

void L6470::resetDevice(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_RESET_DEVICE);
  SPI.endTransaction();
}

uint16_t L6470::getStatus(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_RESET_DEVICE);
  uint16_t statusMSB = transferByte(CMD_NOP) << 8;
  uint16_t statusLSB = transferByte(CMD_NOP);
  SPI.endTransaction();
  return statusMSB | statusLSB;
}

