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
/*
1 tick = 250ns = 250 * 10^-9 = 1 / (4 * 10^6)s
MAX_SPEED -> 1 ~ 1023
MAX_SPEED * 4 * 10^6 * 2^-18 = MAX_SPEED * 15.2587890625 step/s
MIN_SPEED -> 0 ~ 2047
MIN_SPEED * 4 * 10^6 * 2^-24 = MIN_SPEED * 0.2384185791015625 step/s
*/

String directionToString(Direction direction) {
  switch (direction) {
    case Direction::forward: return "forward";
    case Direction::reverse: return "reverse";
  }
  Serial.println("directionToString: ***ASSERT FALSE***"); // TODO: assert false here.
  return "";
}

String motorStateToString(MotorState motorState) {
  switch (motorState) {
    case MotorState::stopped:       return "stopped";
    case MotorState::accelerating:  return "accelerating";
    case MotorState::decelerating:  return "decelerating";
    case MotorState::constantSpeed: return "constantSpeed";
  }
  Serial.println("motorStateToString: ***ASSERT FALSE***"); // TODO: assert false here.
  return "";
}

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
  Serial.println("getLength: ***ASSERT FALSE***"); // TODO: assert false here.
  return 0;
}

void L6470::initialize() {
  SPI.begin();
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);

  resetDevice();
  updateStatus();
}

/* SPI Utility Functions */

uint8_t L6470::transferByte(uint8_t data) {
  digitalWrite(chipSelectPin, LOW);
  uint8_t output = SPI.transfer(data);
  digitalWrite(chipSelectPin, HIGH);
  //Serial.print(data, HEX);
  //Serial.print(" -> ");
  //Serial.print(output, HEX);
  //Serial.print('\n');
  return output;
}

uint32_t L6470::sendBytes(uint32_t value, uint8_t length) {
  uint32_t result = 0;
  while (true) {
    uint8_t shift_amount = ((length - 1) / 8) * 8;
    result |= transferByte((uint8_t) (value >> shift_amount));
    if (shift_amount == 0) break;
    result <<= 8;
    length -= 8;
  }
  return result;
}

/* Raw Commands */

void L6470::setParam(uint8_t param, uint32_t value) {
  uint8_t length = getLength(param);
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_SET_PARAM | (param & 0b00011111));
  sendBytes(value, length);
  SPI.endTransaction();
}

uint32_t L6470::getParam(uint8_t param) {
  uint8_t length = getLength(param);
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_GET_PARAM | (param & 0b00011111));
  uint32_t ret = 0;
  while (true) {
    ret |= transferByte(CMD_NOP);
    if (length <= 8) break;
    ret = ret << 8;
    length -= 8;
  }
  SPI.endTransaction();
  return ret;
}

void L6470::nop(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_NOP);
  SPI.endTransaction();
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

void L6470::goTo(uint32_t absolutePosition) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_GO_TO);
  sendBytes(absolutePosition, 22);
  SPI.endTransaction();
}

void L6470::goToDir(Direction direction, uint32_t absolutePosition) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_GO_TO_DIR | static_cast<uint8_t>(direction));
  sendBytes(absolutePosition, 22);
  SPI.endTransaction();
}

void L6470::goUntil(Action action, Direction direction, uint32_t speed) {
  SPI.beginTransaction(spiSettings);
  transferByte(
      CMD_GO_UNTIL |
      (static_cast<uint8_t>(action) << 3) |
      static_cast<uint8_t>(direction));
  sendBytes(speed, 20);
  SPI.endTransaction();
}

void L6470::releaseSW(Action action, Direction direction) {
  SPI.beginTransaction(spiSettings);
  transferByte(
      CMD_RELEASE_SW |
      (static_cast<uint8_t>(action) << 3) |
      static_cast<uint8_t>(direction));
  SPI.endTransaction();
}

void L6470::goHome(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_GO_HOME);
  SPI.endTransaction();
}

void L6470::goMark(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_GO_MARK);
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

uint16_t L6470::getStatus(void) {
  SPI.beginTransaction(spiSettings);
  transferByte(CMD_GET_STATUS);
  uint16_t statusMSB = transferByte(CMD_NOP) << 8;
  uint16_t statusLSB = transferByte(CMD_NOP);
  SPI.endTransaction();
  return statusMSB | statusLSB;
}

/* Convenience Functions */

#define MAX_OF_MAX_SPEED ((1 << 10) - 1)
void L6470::setMaxStepPerSecond(float speed) {
  uint16_t maxSpeed =
    min((uint16_t) floor(max(0.0, speed) / 15.2587890625), MAX_OF_MAX_SPEED);
  setParam(REG_MAX_SPEED, maxSpeed);
}

#define MAX_OF_MIN_SPEED ((1 << 12) - 1)
void L6470::setMinStepPerSecond(float speed, bool enableOptimization) {
  uint16_t minSpeed =
    min((uint16_t) floor(max(0.0, speed) / 0.2384185791015625), MAX_OF_MIN_SPEED);
  setParam(REG_MIN_SPEED, minSpeed | (enableOptimization ? (1 << 12) : 0));
}

#define MAX_THRESHOLD ((1 << 10) - 1)
void L6470::disableThreshold(void) {
  setParam(REG_FS_SPD, MAX_THRESHOLD);
}

void L6470::setThresholdStepsPerSecond(float speed) {
  uint8_t threshold =
    min((uint8_t) floor(max(0.0, speed) / 15.2587890625 + 0.5), MAX_THRESHOLD);
  setParam(REG_FS_SPD, threshold);
}

void L6470::setKVal(KVal kVal, uint8_t value) {
  uint8_t reg = REG_KVAL_HOLD;
  switch (kVal) {
    case KVal::hold: reg = REG_KVAL_HOLD; break;
    case KVal::run: reg = REG_KVAL_RUN; break;
    case KVal::acc: reg = REG_KVAL_ACC; break;
    case KVal::dec: reg = REG_KVAL_DEC; break;
    default: Serial.println("setKVal: ***ASSERT FALSE***"); // TODO: assert false
  }
  setParam(reg, value);
}

void L6470::setStepMode(StepMode stepMode, bool enableSync, SyncMode syncMode) {
  uint8_t syncEn = enableSync ? 1 << 7 : 0;
  uint8_t syncSel = static_cast<uint8_t>(syncMode) << 4;
  uint8_t stepSel = static_cast<uint8_t>(stepMode);
  setParam(REG_STEP_MODE, syncEn | syncSel | stepSel);
}

void L6470::setConfig(
  OscillatorSelect oscillatorSelect,
  bool disableHardStopInterrupt,
  bool shutdownBridgesOnOvercurrent,
  SlewRate slewRate,
  bool motorSupplyVoltageCompensation,
  PWMFrequencyDivisionFactor pwmFrequencyDivisionFactor,
  PWMFrequencyMultiplicationFactor pwmFrequencyMultiplicationFactor) {
  uint16_t oscSel = static_cast<uint16_t>(oscillatorSelect);
  uint16_t swMode = disableHardStopInterrupt ? 1 << 4 : 0;
  uint16_t ocSD = shutdownBridgesOnOvercurrent ? 1 << 7 : 0;
  uint16_t powSR = static_cast<uint16_t>(slewRate) << 8;
  uint16_t enVSComp = motorSupplyVoltageCompensation ? 1 << 5 : 0;
  uint16_t fPWMInt = static_cast<uint16_t>(pwmFrequencyDivisionFactor) << 13;
  uint16_t fPWMDec = static_cast<uint16_t>(pwmFrequencyMultiplicationFactor) << 10;
  setParam(REG_CONFIG,
      oscSel | swMode | ocSD | powSR | enVSComp | fPWMInt | fPWMDec);
}

void L6470::updateStatus(void) {
  uint16_t status = getStatus();
  currentStatus =
  { .hiZ                 =   status & (1 <<  0)
  , .undervoltageLockout = !(status & (1 <<  9))
  , .thermalWarning      = !(status & (1 << 10))
  , .thermalShutdown     = !(status & (1 << 11))
  , .overcurrent         = !(status & (1 << 12))
  , .stepLossA           =   status & (1 << 13)
  , .stepLossB           =   status & (1 << 14)
  , .commandNotPerformed =   status & (1 <<  7)
  , .commandDoesNotExist =   status & (1 <<  8)
  , .switchClosed        =   status & (1 <<  2)
  , .switchEvent         =   status & (1 <<  3)
  , .busy                = !(status & (1 <<  1))
  , .stepClockMode       =   status & (1 << 15)
  , .direction           =  (status & (1 <<  4)) ? Direction::forward : Direction::reverse
  , .motorState          = static_cast<MotorState>((uint8_t) (status & (0b11 << 5)) >> 5)
  };
}

void L6470::printStatus(void) {
  Serial.print("hiZ:			");
  Serial.println(currentStatus.hiZ);
  Serial.print("undervoltageLockout:		");
  Serial.println(currentStatus.undervoltageLockout);
  Serial.print("thermalWarning:		");
  Serial.println(currentStatus.thermalWarning);
  Serial.print("thermalShutdown:		");
  Serial.println(currentStatus.thermalShutdown);
  Serial.print("overcurrent:		");
  Serial.println(currentStatus.overcurrent);
  Serial.print("stepLossA:			");
  Serial.println(currentStatus.stepLossA);
  Serial.print("stepLossB:			");
  Serial.println(currentStatus.stepLossB);
  Serial.print("commandNotPerformed:	");
  Serial.println(currentStatus.commandNotPerformed);
  Serial.print("commandDoesNotExist:		");
  Serial.println(currentStatus.commandDoesNotExist);
  Serial.print("switchClosed:		");
  Serial.println(currentStatus.switchClosed);
  Serial.print("switchEvent:		");
  Serial.println(currentStatus.switchEvent);
  Serial.print("busy:			");
  Serial.println(currentStatus.busy);
  Serial.print("stepClockMode:		");
  Serial.println(currentStatus.stepClockMode);
  Serial.print("direction:			");
  Serial.println(directionToString(currentStatus.direction));
  Serial.print("motorState:			");
  Serial.println(motorStateToString(currentStatus.motorState));
}
