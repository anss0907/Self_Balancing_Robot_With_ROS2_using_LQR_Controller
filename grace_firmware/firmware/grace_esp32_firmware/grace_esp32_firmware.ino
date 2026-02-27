// =============================================================================
// GRACE Robot — Dual-Core ESP32 Firmware (Torque Control)
// =============================================================================
// Pure relay — ALL intelligence stays in ROS.
//
// Core 0: ROS → STM32   (read torque commands from USB serial, send to hoverboard)
// Core 1: STM32+IMU → ROS (read hoverboard feedback + IMUs, send to USB serial)
//
// ROS → ESP32:  tL<val> tR<val>\n     e.g. "tL120 tR-115\n"
// ESP32 → ROS:  comma-separated sensor string (see sendSensorString)
// =============================================================================

#include <HardwareSerial.h>
#include <Wire.h>
#include <cstring>

// ======================== CONFIG ========================

#define TORQUE_MAX              1000
#define HOVER_SEND_INTERVAL_MS  5      // 200 Hz to STM32 (fast for balancing)
#define SENSOR_SEND_INTERVAL_MS 5      // 200 Hz to ROS
#define DRIVER_TIMEOUT_MS       500
#define USB_BAUD                115200
#define HOVER_BAUD              115200
#define SERIAL_PARSE_TIMEOUT_MS 5
#define HOVER_RX_PIN            16
#define HOVER_TX_PIN            17
#define I2C_SDA_PIN             21
#define I2C_SCL_PIN             22
#define IMU1_ADDR               0x68
#define IMU2_ADDR               0x69
#define CORE0_STACK_SIZE        4096
#define CORE1_STACK_SIZE        4096

static constexpr uint16_t START_FRAME = 0xABCD;

// ======================== STRUCTS ========================

struct ImuRawData {
  int16_t ax, ay, az, gx, gy, gz;
};

// Sent TO hoverboard (binary, unchanged protocol)
struct HoverCommand {
  uint16_t start;
  int16_t  steer;     // left torque (TANK_STEERING)
  int16_t  speed;     // right torque (TANK_STEERING)
  uint16_t checksum;
};

// Received FROM hoverboard (extended torque-control firmware)
struct HoverFeedback {
  uint16_t start;
  int16_t  cmd1, cmd2;
  int16_t  speedR_meas, speedL_meas;
  int16_t  batVoltage, boardTemp;
  uint16_t cmdLed;
  int16_t  curL_DC, curR_DC;
  int16_t  iqL, iqR;
  int16_t  idL, idR;
  int16_t  angleL, angleR;
  uint16_t checksum;
};

// Shared between cores — protected by mutex
struct SharedState {
  // Core 0 writes, Core 1 reads
  int16_t  targetTorqueL;
  int16_t  targetTorqueR;

  // Core 1 writes, Core 0 reads
  int16_t  measuredSpeedL, measuredSpeedR;
  int16_t  batteryMv, boardTempDeciC;
  int16_t  curL_DC, curR_DC;
  int16_t  iqL, iqR;
  int16_t  idL, idR;
  int16_t  angleL, angleR;
  bool     driverAlive;
  uint32_t lastDriverFeedbackMs;
  ImuRawData imu1, imu2;
  bool     imu1Ok, imu2Ok;
};

// ======================== GLOBALS ========================

HardwareSerial HoverSerial(2);
SemaphoreHandle_t stateMutex;
SharedState shared = {};

// ======================== IMU ========================

static bool imuWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool imuReadRaw(uint8_t addr, ImuRawData &out) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)14) != 14) return false;
  out.ax = (Wire.read() << 8) | Wire.read();
  out.ay = (Wire.read() << 8) | Wire.read();
  out.az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temp
  out.gx = (Wire.read() << 8) | Wire.read();
  out.gy = (Wire.read() << 8) | Wire.read();
  out.gz = (Wire.read() << 8) | Wire.read();
  return true;
}

static bool initImu(uint8_t addr) {
  if (!imuWriteReg(addr, 0x6B, 0x00)) return false;
  delay(10);
  return true;
}

// ======================== HOVERBOARD SEND ========================

static void sendHoverCommand(int16_t torqueL, int16_t torqueR) {
  torqueL = constrain(torqueL, -TORQUE_MAX, TORQUE_MAX);
  torqueR = constrain(torqueR, -TORQUE_MAX, TORQUE_MAX);

  HoverCommand cmd;
  cmd.start    = START_FRAME;
  cmd.steer    = torqueL;
  cmd.speed    = torqueR;
  cmd.checksum = cmd.start ^ cmd.steer ^ cmd.speed;
  HoverSerial.write(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}

// ======================== SENSOR STRING ========================

static void printField(const char* label, int16_t value, int width) {
  Serial.print(label);
  Serial.printf("%0*d", width, abs(value));
  Serial.print(value >= 0 ? 'p' : 'n');
}

static void sendSensorString(const SharedState &s, uint32_t nowMs) {
  static uint32_t lastSendTime = 0;
  uint32_t dt = nowMs - lastSendTime;
  lastSendTime = nowMs;

  Serial.print(',');
  Serial.print("T"); Serial.printf("%04lu", dt);           Serial.print(',');
  Serial.print("D"); Serial.print(s.driverAlive ? '1' : '0'); Serial.print(',');

  printField("b", s.batteryMv, 5);        Serial.print(',');
  printField("t", s.boardTempDeciC, 4);   Serial.print(',');

  printField("TL", s.targetTorqueL, 4);   Serial.print(',');
  printField("Lm", s.measuredSpeedL, 4);  Serial.print(',');
  printField("TR", s.targetTorqueR, 4);   Serial.print(',');
  printField("Rm", s.measuredSpeedR, 4);  Serial.print(',');

  printField("cL", s.curL_DC, 5);         Serial.print(',');
  printField("cR", s.curR_DC, 5);         Serial.print(',');
  printField("qL", s.iqL, 5);            Serial.print(',');
  printField("qR", s.iqR, 5);            Serial.print(',');
  printField("dL", s.idL, 5);            Serial.print(',');
  printField("dR", s.idR, 5);            Serial.print(',');
  printField("aL", s.angleL, 5);         Serial.print(',');
  printField("aR", s.angleR, 5);         Serial.print(',');

  Serial.print("I1"); Serial.print(s.imu1Ok ? '1' : '0'); Serial.print(',');
  printField("ax1", s.imu1.ax, 5); Serial.print(',');
  printField("ay1", s.imu1.ay, 5); Serial.print(',');
  printField("az1", s.imu1.az, 5); Serial.print(',');
  printField("gx1", s.imu1.gx, 5); Serial.print(',');
  printField("gy1", s.imu1.gy, 5); Serial.print(',');
  printField("gz1", s.imu1.gz, 5); Serial.print(',');

  Serial.print("I2"); Serial.print(s.imu2Ok ? '1' : '0'); Serial.print(',');
  printField("ax2", s.imu2.ax, 5); Serial.print(',');
  printField("ay2", s.imu2.ay, 5); Serial.print(',');
  printField("az2", s.imu2.az, 5); Serial.print(',');
  printField("gx2", s.imu2.gx, 5); Serial.print(',');
  printField("gy2", s.imu2.gy, 5); Serial.print(',');
  printField("gz2", s.imu2.gz, 5);

  Serial.println();
}

// ======================== CORE 0: ROS → STM32 ========================
// ONE JOB: read torque commands from USB serial, forward to hoverboard
//
// LINE-BUFFER approach: accumulate bytes until '\n', then parse the
// complete line.  This avoids the timing bug where byte-by-byte USB
// arrival caused Serial.available() to return false between 't' and
// the motor letter.
//
// Accepted formats (case-insensitive, any whitespace between tokens):
//   tL100 tR-200\n          ← from ROS / Serial Monitor
//   tL100\n                 ← single motor update is fine too

#define CMD_BUF_SIZE 64

void core0Task(void *pvParameters) {
  (void)pvParameters;
  uint32_t lastHoverSend = 0;

  char cmdBuf[CMD_BUF_SIZE];
  uint8_t cmdIdx = 0;

  for (;;) {
    uint32_t now = millis();

    // ---- Accumulate characters into line buffer ----
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (cmdIdx > 0) {
          cmdBuf[cmdIdx] = '\0';

          // ---- Parse the complete line ----
          // Walk through the buffer looking for 't'/'T' tokens
          char *p = cmdBuf;
          while (*p) {
            // Skip whitespace
            while (*p == ' ' || *p == '\t') p++;
            if (*p == '\0') break;

            if (*p == 't' || *p == 'T') {
              p++;  // skip 't'
              char motor = *p;
              if (motor == '\0') break;
              p++;  // skip motor letter

              // Parse the integer value
              int16_t torque = (int16_t)constrain(atoi(p), -TORQUE_MAX, TORQUE_MAX);

              // Advance past the number (optional sign + digits)
              if (*p == '-' || *p == '+') p++;
              while (*p >= '0' && *p <= '9') p++;

              if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                if (motor == 'L' || motor == 'l') shared.targetTorqueL = torque;
                if (motor == 'R' || motor == 'r') shared.targetTorqueR = torque;
                xSemaphoreGive(stateMutex);
              }
            } else {
              p++;  // skip unknown character
            }
          }

          // Send to STM32 RIGHT NOW after parsing the full line
          int16_t cmdL = 0, cmdR = 0;
          if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            cmdL = shared.targetTorqueL;
            cmdR = shared.targetTorqueR;
            xSemaphoreGive(stateMutex);
          }
          sendHoverCommand(cmdL, cmdR);
          lastHoverSend = now;
        }
        cmdIdx = 0;  // reset for next line
      } else {
        if (cmdIdx < CMD_BUF_SIZE - 1) {
          cmdBuf[cmdIdx++] = c;
        }
        // else: overflow → silently drop extra chars until newline
      }
    }

    // STM32 requires periodic commands to stay alive (handshake).
    // Send every 5ms even when no new data from ROS.
    if (now - lastHoverSend >= HOVER_SEND_INTERVAL_MS) {
      int16_t cmdL = 0, cmdR = 0;
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        cmdL = shared.targetTorqueL;
        cmdR = shared.targetTorqueR;
        xSemaphoreGive(stateMutex);
      }
      sendHoverCommand(cmdL, cmdR);
      lastHoverSend = now;
    }

    taskYIELD();
  }
}

// ======================== CORE 1: STM32+IMU → ROS ========================
// ONE JOB: read hoverboard feedback + IMUs, send sensor string to ROS

void core1Task(void *pvParameters) {
  (void)pvParameters;

  // Hoverboard receive state machine
  uint8_t idx = 0;
  byte incomingByte = 0, incomingBytePrev = 0;
  byte *p = nullptr;
  HoverFeedback newFb = {};

  // Init I2C + IMUs
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(50);
  bool imu1Ok = initImu(IMU1_ADDR);
  bool imu2Ok = initImu(IMU2_ADDR);

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    shared.imu1Ok = imu1Ok;
    shared.imu2Ok = imu2Ok;
    xSemaphoreGive(stateMutex);
  }

  uint32_t lastSensorSend = 0;

  for (;;) {
    uint32_t now = millis();

    // ---- Read hoverboard feedback ----
    while (HoverSerial.available()) {
      incomingByte = HoverSerial.read();
      uint16_t frame = (static_cast<uint16_t>(incomingByte) << 8) | incomingBytePrev;

      if (frame == START_FRAME) {
        p = reinterpret_cast<byte*>(&newFb);
        *p++ = incomingBytePrev;
        *p++ = incomingByte;
        idx = 2;
      } else if (idx >= 2 && idx < sizeof(HoverFeedback)) {
        if (p) { *p++ = incomingByte; idx++; }
        else   { idx = 0; }
      }

      if (idx == sizeof(HoverFeedback)) {
        uint16_t cs = newFb.start ^ newFb.cmd1 ^ newFb.cmd2 ^
                      newFb.speedR_meas ^ newFb.speedL_meas ^
                      newFb.batVoltage ^ newFb.boardTemp ^ newFb.cmdLed ^
                      newFb.curL_DC ^ newFb.curR_DC ^
                      newFb.iqL ^ newFb.iqR ^
                      newFb.idL ^ newFb.idR ^
                      newFb.angleL ^ newFb.angleR;

        if (newFb.start == START_FRAME && cs == newFb.checksum) {
          if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared.measuredSpeedL  = newFb.speedL_meas;
            shared.measuredSpeedR  = newFb.speedR_meas;
            shared.batteryMv       = newFb.batVoltage;
            shared.boardTempDeciC  = newFb.boardTemp;
            shared.curL_DC         = newFb.curL_DC;
            shared.curR_DC         = newFb.curR_DC;
            shared.iqL             = newFb.iqL;
            shared.iqR             = newFb.iqR;
            shared.idL             = newFb.idL;
            shared.idR             = newFb.idR;
            shared.angleL          = newFb.angleL;
            shared.angleR          = newFb.angleR;
            shared.driverAlive     = true;
            shared.lastDriverFeedbackMs = now;
            xSemaphoreGive(stateMutex);
          }
        }
        idx = 0;
      }
      incomingBytePrev = incomingByte;
    }

    // ---- Read IMUs ----
    ImuRawData d1 = {}, d2 = {};
    bool r1 = false, r2 = false;
    if (imu1Ok) { r1 = imuReadRaw(IMU1_ADDR, d1); if (!r1) imu1Ok = false; }
    if (imu2Ok) { r2 = imuReadRaw(IMU2_ADDR, d2); if (!r2) imu2Ok = false; }

    if (r1 || r2) {
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        if (r1) shared.imu1 = d1;
        if (r2) shared.imu2 = d2;
        shared.imu1Ok = imu1Ok;
        shared.imu2Ok = imu2Ok;
        xSemaphoreGive(stateMutex);
      }
    }

    // ---- Driver timeout ----
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      if (shared.driverAlive && (now - shared.lastDriverFeedbackMs) >= DRIVER_TIMEOUT_MS) {
        shared.driverAlive = false;
        shared.measuredSpeedL = shared.measuredSpeedR = 0;
        shared.batteryMv = shared.boardTempDeciC = 0;
        shared.curL_DC = shared.curR_DC = 0;
        shared.iqL = shared.iqR = shared.idL = shared.idR = 0;
        shared.angleL = shared.angleR = 0;
      }
      xSemaphoreGive(stateMutex);
    }

    // ---- Send sensor string to ROS ----
    if (now - lastSensorSend >= SENSOR_SEND_INTERVAL_MS) {
      SharedState snap;
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        snap = shared;
        xSemaphoreGive(stateMutex);
      }
      sendSensorString(snap, now);
      lastSensorSend = now;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======================== SETUP ========================

void setup() {
  Serial.begin(USB_BAUD);
  Serial.setTimeout(SERIAL_PARSE_TIMEOUT_MS);
  delay(100);

  HoverSerial.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);
  delay(100);
  while (HoverSerial.available()) HoverSerial.read();

  stateMutex = xSemaphoreCreateMutex();
  shared = {};

  // Core 0: ROS → STM32
  xTaskCreatePinnedToCore(core0Task, "ROS_to_STM32", CORE0_STACK_SIZE, NULL, 1, NULL, 0);

  // Core 1: STM32+IMU → ROS
  xTaskCreatePinnedToCore(core1Task, "HW_to_ROS", CORE1_STACK_SIZE, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
