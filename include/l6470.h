#ifndef L6470_H
#define L6470_H

#include "SPI.h"

#define REG_ABS_POS    0x01 // 22 bits
#define REG_EL_POS     0x02 //  9 bits
#define REG_MARK       0x03 // 22 bits
#define REG_SPEED      0x04 // 20 bits
#define REG_ACC        0x05 // 12 bits
#define REG_DEC        0x06 // 12 bits
#define REG_MAX_SPEED  0x07 // 10 bits
#define REG_MIN_SPEED  0x08 // 13 bits
#define REG_FS_SPD     0x15 // 10 bits
#define REG_KVAL_HOLD  0x09 //  8 bits
#define REG_KVAL_RUN   0x0a //  8 bits
#define REG_KVAL_ACC   0x0b //  8 bits
#define REG_KVAL_DEC   0x0c //  8 bits
#define REG_INT_SPD    0x0d // 14 bits
#define REG_ST_SLP     0x0e //  8 bits
#define REG_FN_SLP_ACC 0x0f //  8 bits
#define REG_FN_SLP_DEC 0x10 //  8 bits
#define REG_K_THERM    0x11 //  4 bits
#define REG_ADC_OUT    0x12 //  5 bits
#define REG_OCD_TH     0x13 //  4 bits
#define REG_STALL_TH   0x14 //  7 bits
#define REG_STEP_MODE  0x16 //  8 bits
#define REG_ALARM_EN   0x17 //  8 bits
#define REG_CONFIG     0x18 // 16 bits
#define REG_STATUS     0x19 // 16 bits

#define REG_ABS_POS_LEN    22
#define REG_EL_POS_LEN      9
#define REG_MARK_LEN       22
#define REG_SPEED_LEN      20
#define REG_ACC_LEN        12
#define REG_DEC_LEN        12
#define REG_MAX_SPEED_LEN  10
#define REG_MIN_SPEED_LEN  13
#define REG_FS_SPD_LEN     10
#define REG_KVAL_HOLD_LEN   8
#define REG_KVAL_RUN_LEN    8
#define REG_KVAL_ACC_LEN    8
#define REG_KVAL_DEC_LEN    8
#define REG_INT_SPD_LEN    14
#define REG_ST_SLP_LEN      8
#define REG_FN_SLP_ACC_LEN  8
#define REG_FN_SLP_DEC_LEN  8
#define REG_K_THERM_LEN     4
#define REG_ADC_OUT_LEN     5
#define REG_OCD_TH_LEN      4
#define REG_STALL_TH_LEN    7
#define REG_STEP_MODE_LEN   8
#define REG_ALARM_EN_LEN    8
#define REG_CONFIG_LEN     16
#define REG_STATUS_LEN     16

#define CMD_NOP          0b00000000
#define CMD_SET_PARAM    0b00000000
#define CMD_GET_PARAM    0b00100000
#define CMD_RUN          0b01010000
#define CMD_STEP_CLOCK   0b01011000
#define CMD_MOVE         0b01000000
#define CMD_GO_TO        0b01100000
#define CMD_GO_TO_DIR    0b01101000
#define CMD_GO_UNTIL     0b10000010
#define CMD_RELEASE_SW   0b10010010
#define CMD_GO_HOME      0b01110000
#define CMD_GO_MARK      0b01111000
#define CMD_RESET_POS    0b11011000
#define CMD_RESET_DEVICE 0b11000000
#define CMD_SOFT_STOP    0b10110000
#define CMD_HARD_STOP    0b10111000
#define CMD_SOFT_HI_Z    0b10100000
#define CMD_HARD_HI_Z    0b10101000
#define CMD_GET_STATUS   0b11010000

enum class Direction : uint8_t {
  forward = 1,
  reverse = 0
};

String directionToString(Direction direction);

enum class Action : uint8_t {
  reset = 0,
  copy = 1
};

enum class StepMode : uint8_t {
  fullStep      = 0b000,
  halfStep      = 0b001,
  microstep_4   = 0b010,
  microstep_8   = 0b011,
  microstep_16  = 0b100,
  microstep_32  = 0b101,
  microstep_64  = 0b110,
  microstep_128 = 0b111
};

enum class SyncMode : uint8_t {
  half     = 0b000,
  times_1  = 0b001,
  times_2  = 0b010,
  times_4  = 0b011,
  times_8  = 0b100,
  times_16 = 0b101,
  times_32 = 0b110,
  times_64 = 0b111,
};

enum class KVal {
  hold,
  run,
  acc,
  dec
};

enum class SlewRate : uint8_t {
  vpus_180 = 0b00,
  vpus_290 = 0b10,
  vpus_530 = 0b11,
};

enum class OscillatorSelect : uint8_t {
  internal16MHzOutNone   = 0b0000,
  internal16MHzOut2MHz   = 0b1000,
  internal16MHzOut4MHz   = 0b1001,
  internal16MHzOut8MHz   = 0b1010,
  internal16MHzOut16MHz  = 0b1011,
  external8MHz           = 0b0100,
  external16MHz          = 0b0101,
  external32MHz          = 0b0110,
  external64MHz          = 0b0111,
  external8MHzOutInvert  = 0b1100,
  external16MHzOutInvert = 0b1101,
  external32MHzOutInvert = 0b1110,
  external64MHzOutInvert = 0b1111
};

enum class PWMFrequencyDivisionFactor : uint8_t {
  by1 = 0b000,
  by2 = 0b001,
  by3 = 0b010,
  by4 = 0b011,
  by5 = 0b100,
  by6 = 0b101,
  by7 = 0b110,
};

enum class PWMFrequencyMultiplicationFactor : uint8_t {
  by0_625 = 0b000,
  by0_75  = 0b001,
  by0_875 = 0b010,
  by1     = 0b011,
  by1_25  = 0b100,
  by1_5   = 0b101,
  by1_75  = 0b110,
  by2     = 0b111,
};

enum class MotorState : uint8_t {
  stopped = 0b00,
  accelerating = 0b01,
  decelerating = 0b10,
  constantSpeed = 0b11
};

String motorStateToString(MotorState motorState);

struct Status {
  bool hiZ;                 // HiZ
  bool undervoltageLockout; // UVLO
  bool thermalWarning;      // TH_WRN
  bool thermalShutdown;     // TH_SD
  bool overcurrent;         // OCD
  bool stepLossA;           // STEP_LOSS_A
  bool stepLossB;           // STEP_LOSS_B
  bool commandNotPerformed; // NOTPERF_CMD
  bool commandDoesNotExist; // WRONG_CMD
  bool switchClosed;        // SW_F
  bool switchEvent;         // SW_EVN
  bool busy;                // BUSY
  bool stepClockMode;       // SCK_MOD
  Direction direction;      // DIR
  MotorState motorState;    // MOT_STATE
};
 // TODO: naming

class L6470 {
    const int chipSelectPin;
    const SPISettings spiSettings;
    uint8_t transferByte(uint8_t data);
    uint32_t sendBytes(uint32_t value, uint8_t length);
  public:
    Status currentStatus;
    L6470(int chipSelectPin);
    uint8_t getLength(uint8_t param);
    void initialize(void);

    /* Raw Commands */

    void setParam(uint8_t param, uint32_t value);
    uint32_t getParam(uint8_t param);
    void nop(void);
    void run(Direction direction, uint32_t speed);
    void stepClock(Direction direction);
    void move(Direction direction, uint32_t steps);
    void goTo(uint32_t absolutePosition);
    void goToDir(Direction direction, uint32_t absolutePosition);
    void goUntil(Action action, Direction direction, uint32_t speed);
    void releaseSW(Action action, Direction direction);
    void goHome(void);
    void goMark(void);
    void resetPos(void);
    void resetDevice(void);
    void softStop(void);
    void hardStop(void);
    void softHiZ(void);
    void hardHiZ(void);
    uint16_t getStatus(void);

    /* Convenience Functions */

    void setStepMode(
        StepMode stepMode,
        bool enableSync = false,
        SyncMode syncMode = SyncMode::half);
    void setMaxStepPerSecond(float speed);
    void setMinStepPerSecond(float speed, bool enableOptimization = false);
    void disableThreshold(void);
    void setThresholdStepsPerSecond(float speed);
    void setKVal(KVal kVal, uint8_t value);
    void setConfig(
        OscillatorSelect oscillatorSelect,
        bool disableHardStopInterrupt,
        bool shutdownBridgesOnOvercurrent,
        SlewRate slewRate,
        bool motorSupplyVoltageCompensation,
        PWMFrequencyDivisionFactor pwmFrequencyDivisionFactor,
        PWMFrequencyMultiplicationFactor pwmFrequencyMultiplicationFactor);
    void updateStatus(void);
    void printStatus(void);
};

#endif
