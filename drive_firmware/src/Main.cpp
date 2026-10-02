#include <Arduino.h>
#include <VescUart.h>
#include <Wire.h>

#include "MicroROS.h"
#include "Messages.h"

// #include <urc_msgs/msg/tank_drive_target.h>

#define NODE_NAME "pdb_teensy"
#define NAMESPACE ""

#define MAX_RPM 50
#define RPM_TO_ERPM (11.0 * 42.875)

VescUart VESC1; // Port 1 - left side
VescUart VESC2; // Port 2 - right side

// ----------------------- MCP9801 -----------------------
static constexpr uint8_t MCP9801_ADDR   = 0x48;
static constexpr uint8_t MCP9801_REG_TA = 0x00;

// ----------------------- ADC ---------------------------
static constexpr float ADC_VREF = 3.3f;
static constexpr int   ADC_BITS = 12;
static constexpr float ADC_MAX  = (1 << ADC_BITS) - 1;

// NOTE: Set these to match your *actual* divider.
// Your earlier schematic (250k/50k) will overdrive A0 at 25V.
// Keep these as the "safe" example until hardware is corrected.
static constexpr float R_TOP = 330000.0f; // example safe choice
static constexpr float R_BOT = 50000.0f;

// ----------------------- INA240 ------------------------
// If you keep INA240A3 with both refs to GND:
static constexpr float INA_GAIN = 100.0f; // A3
static constexpr float INA_VREF = 0.0f;   // refs to GND
static constexpr float R_SHUNT  = 0.001f; // 1 mΩ

// ----------------------- RGB LED pins ------------------
// MOSFET gates
static constexpr int LED_R_PIN = 33;
static constexpr int LED_G_PIN = 36;
static constexpr int LED_B_PIN = 37;

// ----------------------- Fan pin -----------------------
static constexpr int FAN_PIN = 13;        // FAN MOSFET gate

// PWM resolution for LED/Fan control
static constexpr int PWM_BITS = 12;
static constexpr int PWM_MAX  = (1 << PWM_BITS) - 1;


// ----------------------- Fan control curve --------------
// Tune these:
static constexpr float FAN_OFF_C  = 10.0f;  // <= this: fan off
static constexpr float FAN_FULL_C = 28.0f;  // >= this: fan full
static constexpr float FAN_MIN_PWM = 0.20f; // when "on", minimum duty to overcome stall (tune or set 0)

// Optional: limit update jitter
static constexpr float FAN_SLEW_PER_SEC = 1.5f; // max change in PWM per second (0..1). Set 0 to disable.


// ----------------------- ROS messages/pubs --------------
int i = 0;

FloatMsg testMsg;
uROS::Publisher p("drive_test", &testMsg);

StringMsg logMsg;
uROS::Publisher LogPub("/pdb_teensy/log", &logMsg);

FloatMsg tempMsg;
uROS::Publisher TempPub("mcp9801_temp_c", &tempMsg);

FloatMsg battVMsg, currAMsg;
uROS::Publisher BattVPub("battery_voltage_v", &battVMsg);
uROS::Publisher CurrPub("battery_current_a", &currAMsg);

uROS::Publisher TestPub("drive_echo", &testMsg);

// ----------------------- LED control state --------------
volatile float led_r = 0.0f; // 0..1
volatile float led_g = 0.0f; // 0..1
volatile float led_b = 0.0f; // 0..1

// ----------------------- Fan control state --------------
// Fan state
static float fan_pwm = 0.0f;          // applied (0..1)
static float fan_pwm_target = 0.0f;   // desired (0..1)
static uint32_t last_fan_update_ms = 0;

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static inline int pwmFrom01(float x) {
  return (int)(clamp01(x) * PWM_MAX + 0.5f);
}

static void ApplyLedPWM() {
  analogWrite(LED_R_PIN, pwmFrom01(led_r));
  analogWrite(LED_G_PIN, pwmFrom01(led_g));
  analogWrite(LED_B_PIN, pwmFrom01(led_b));
}

static void ApplyFanPWM(float duty01) {
  analogWrite(FAN_PIN, pwmFrom01(duty01));
}

static float FanTargetFromTemp(float tempC) {
  if (tempC <= FAN_OFF_C) return 0.0f;
  if (tempC >= FAN_FULL_C) return 1.0f;

  float t = (tempC - FAN_OFF_C) / (FAN_FULL_C - FAN_OFF_C); // 0..1
  float out = t;

  // Ensure we don't command a too-low duty that stalls the fan
  if (out > 0.0f && out < FAN_MIN_PWM) out = FAN_MIN_PWM;
  return clamp01(out);
}


static void UpdateFanFromTarget(float target01) {
  uint32_t now = millis();
  float dt = (now - last_fan_update_ms) / 1000.0f;
  if (last_fan_update_ms == 0) dt = 0.0f;
  last_fan_update_ms = now;

  fan_pwm_target = clamp01(target01);

  if (FAN_SLEW_PER_SEC <= 0.0f || dt <= 0.0f) {
    fan_pwm = fan_pwm_target;
  } else {
    float max_step = FAN_SLEW_PER_SEC * dt;
    float diff = fan_pwm_target - fan_pwm;
    if (diff >  max_step) diff =  max_step;
    if (diff < -max_step) diff = -max_step;
    fan_pwm = clamp01(fan_pwm + diff);
  }

  ApplyFanPWM(fan_pwm);
}

// Subscribers for LED channels (Float32 0..1)
FloatMsg ledRMsg, ledGMsg, ledBMsg;

uROS::Subscriber LedRSub("led_r", ledRMsg, [](const void*) {
  led_r = clamp01(ledRMsg._msg.data);
  ApplyLedPWM();
});

uROS::Subscriber LedGSub("led_g", ledGMsg, [](const void*) {
  led_g = clamp01(ledGMsg._msg.data);
  ApplyLedPWM();
});

uROS::Subscriber LedBSub("led_b", ledBMsg, [](const void*) {
  led_b = clamp01(ledBMsg._msg.data);
  ApplyLedPWM();
});

// ----------------------- Helpers ------------------------
static inline float adcVolts(int pin) {
  int raw = analogRead(pin);
  return (raw * ADC_VREF) / ADC_MAX;
}

static bool ReadMCP9801TempC(float &out_c) {
  Wire.beginTransmission(MCP9801_ADDR);
  Wire.write(MCP9801_REG_TA);
  if (Wire.endTransmission(false) != 0) return false;

  int n = Wire.requestFrom((int)MCP9801_ADDR, 2);
  if (n != 2) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  int16_t raw = (int16_t)((msb << 8) | lsb);
  out_c = (float)raw / 16.0f;
  return true;
}

static void StartupLedSequence() {
  // 3 red flashes
  for (int k = 0; k < 3; k++) {
    led_r = 1.0f; led_g = 0.0f; led_b = 0.0f;
    ApplyLedPWM();
    delay(250);

    led_r = 0.0f; led_g = 0.0f; led_b = 0.0f;
    ApplyLedPWM();
    delay(250);
  }

  // Solid green after startup
  led_r = 0.0f;
  led_g = 1.0f;
  led_b = 0.0f;
  ApplyLedPWM();
}

bool update = false;

IntMsg leftSetDriveMsg;
uROS::Subscriber LeftDriveSub("/drive/left_set_rpm", leftSetDriveMsg);

FloatMsg leftDriveRpmMsg;
uROS::Publisher LeftDrivePub("/drive/left_get_rpm", &leftDriveRpmMsg);

// FloatMsg leftFrontRpmMsg;
// uROS::Publisher LeftFrontRpmPub("/drive/left_front_rpm", &leftFrontRpmMsg);

// FloatMsg leftBackRpmMsg;
// uROS::Publisher LeftBackRpmPub("/drive/left_back_rpm", &leftBackRpmMsg);

IntMsg rightSetDriveMsg;
uROS::Subscriber RightDriveSub("/drive/right_set_rpm", rightSetDriveMsg);

FloatMsg rightDriveRpmMsg;
uROS::Publisher RightDrivePub("/drive/right_get_rpm", &rightDriveRpmMsg);

// FloatMsg rightFrontRpmMsg;
// uROS::Publisher RightFrontRpmPub("/drive/right_front_rpm", &rightFrontRpmMsg);

// FloatMsg rightBackRpmMsg;
// uROS::Publisher RightBackRpmPub("/drive/right_back_rpm", &rightBackRpmMsg);

void setup() {

  // ADC read resolution
  analogReadResolution(ADC_BITS);

  // PWM resolution (shared for LED + fan)
  analogWriteResolution(PWM_BITS);

  // MOSFET gate pins
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  // Start with LED + fan off
  led_r = led_g = led_b = 0.0f;
  ApplyLedPWM();
  ApplyFanPWM(0.0f);

  StartupLedSequence();

  set_microros_transports();
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    led_r = 0.5;
    led_g = led_b = 0.0f;
    ApplyLedPWM();
    float tempC;
    if (ReadMCP9801TempC(tempC)) {
      float target = FanTargetFromTemp(tempC);
      UpdateFanFromTarget(target);
    } else {
      UpdateFanFromTarget(1.0f);
    }
    delay(250);
    led_r = 0.0;
    ApplyLedPWM();
    // delay(500);
  }

  led_r = 0.0;
  led_g = 0.0;
  led_b = 0.5;
  ApplyLedPWM();

  uROS::Init_MicroROS(NODE_NAME, NAMESPACE);

  led_r = led_g = led_b = 0.5;
  ApplyLedPWM();

  leftSetDriveMsg.Init();
  leftDriveRpmMsg.Init();
  // leftFrontRpmMsg.Init();
  // leftBackRpmMsg.Init();
  rightSetDriveMsg.Init();
  rightDriveRpmMsg.Init();
  // rightFrontRpmMsg.Init();
  // rightBackRpmMsg.Init();

  LeftDriveSub.Init(&uROS::QOS_Teleop);
  LeftDrivePub.Init(&uROS::QOS_Teleop);
  // LeftFrontRpmPub.Init(&uROS::QOS_Teleop);
  // LeftBackRpmPub.Init(&uROS::QOS_Teleop);
  RightDriveSub.Init(&uROS::QOS_Teleop);
  RightDrivePub.Init(&uROS::QOS_Teleop);
  // RightFrontRpmPub.Init(&uROS::QOS_Teleop);
  // RightBackRpmPub.Init(&uROS::QOS_Teleop);

  Serial1.begin(115200, SERIAL_8N1);
  Serial2.begin(115200, SERIAL_8N1);
  VESC1.setSerialPort(&Serial1);
  VESC2.setSerialPort(&Serial2);

  // IMPORTANT: removed tone(13, ...) because pin 13 is FAN gate now.

  // I2C init
  Wire.begin();
  Wire.setClock(400000);

  // uROS::Init_MicroROS(NODE_NAME, NAMESPACE);

  p.Init();
  logMsg.Init();
  LogPub.Init();

  TempPub.Init();
  BattVPub.Init();
  CurrPub.Init();
  TestPub.Init();

  // Subscribers
  ledRMsg.Init();
  ledGMsg.Init();
  ledBMsg.Init();

  LedRSub.Init();
  LedGSub.Init();
  LedBSub.Init();

  delay(250);
  logMsg.Set("[INFO] uROS initialized");
  LogPub.Publish();

  float t;
  if (ReadMCP9801TempC(t)) {
    logMsg.Set("[INFO] MCP9801 read OK");
  } else {
    logMsg.Set("[WARN] MCP9801 read FAILED (check address/wiring)");
  }
  LogPub.Publish();

  logMsg.Set("[INFO] LED topics: led_r, led_g, led_b (Float32 0..1)");
  LogPub.Publish();

  logMsg.Set("[INFO] Fan topic: fan_pwm (Float32 0..1)");
  LogPub.Publish();
}

unsigned long lastUpdate = 0;

void loop() {
  if (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
    SCB_AIRCR = 0x05FA0004;
  }

  uROS::Spin(0);

  int32_t leftRPM = leftSetDriveMsg.GetValue();
  int32_t rightRPM = rightSetDriveMsg.GetValue();
  if (leftRPM < -MAX_RPM) {
    leftRPM = -MAX_RPM;
  } else if (leftRPM > MAX_RPM) {
    leftRPM = MAX_RPM;
  }
  if (rightRPM < -MAX_RPM) {
    rightRPM = -MAX_RPM;
  } else if (rightRPM > MAX_RPM) {
    rightRPM = MAX_RPM;
  }

  // 12 pole pairs to convert to ERPM
  VESC1.setRPM(leftRPM * RPM_TO_ERPM * -1.0, 0);
  VESC1.setRPM(leftRPM * RPM_TO_ERPM * 1.0, 1);
  VESC2.setRPM(rightRPM * RPM_TO_ERPM * 1.0, 0);
  VESC2.setRPM(rightRPM * RPM_TO_ERPM * 1.0, 1);

  leftDriveRpmMsg._msg.data = 0.0;
  if (VESC1.getVescValues(0)) {
    leftDriveRpmMsg._msg.data += VESC1.data.rpm / RPM_TO_ERPM / 2.0 * -1;
  }
  if (VESC1.getVescValues(1)) {
    leftDriveRpmMsg._msg.data += VESC1.data.rpm / RPM_TO_ERPM / 2.0 * 1;
  }
  LeftDrivePub.Publish();

  rightDriveRpmMsg._msg.data = 0.0;
  if (VESC2.getVescValues(0)) {
    rightDriveRpmMsg._msg.data += VESC2.data.rpm / RPM_TO_ERPM / 2.0;
  }
  if (VESC2.getVescValues(1)) {
    rightDriveRpmMsg._msg.data += VESC2.data.rpm / RPM_TO_ERPM / 2.0;
  }
  RightDrivePub.Publish();

  // leftBackRpmMsg._msg.data = 0.0;
  // if (VESC1.getVescValues(0)) {
  //   leftBackRpmMsg._msg.data = VESC1.data.rpm / DRIVE_POLE_PAIRS;
  // }
  // LeftBackRpmPub.Publish();
  // leftFrontRpmMsg._msg.data = 0.0;
  // if (VESC1.getVescValues(1)) {
  //   leftFrontRpmMsg._msg.data = VESC1.data.rpm / DRIVE_POLE_PAIRS;
  // }
  // LeftFrontRpmPub.Publish();

  // rightBackRpmMsg._msg.data = 0.0;
  // if (VESC2.getVescValues(0)) {
  //   rightBackRpmMsg._msg.data = VESC2.data.rpm / DRIVE_POLE_PAIRS;
  // }
  // RightBackRpmPub.Publish();
  // rightFrontRpmMsg._msg.data = 0.0;
  // if (VESC2.getVescValues(1)) {
  //   rightFrontRpmMsg._msg.data = VESC2.data.rpm / DRIVE_POLE_PAIRS;
  // }
  // RightFrontRpmPub.Publish();

  
  // Voltage (A0)
  float v_adc0 = adcVolts(A0);
  float v_batt = v_adc0 * (R_TOP + R_BOT) / R_BOT;
  battVMsg._msg.data = v_batt;
  BattVPub.Publish();

  // Current (A1)
  float v_out   = adcVolts(A1);
  float v_shunt = (v_out - INA_VREF) / INA_GAIN;
  float i_batt  = v_shunt / R_SHUNT;
  currAMsg._msg.data = i_batt;
  CurrPub.Publish();

  // Temperature
  float tempC;
  if (ReadMCP9801TempC(tempC)) {
    tempMsg._msg.data = tempC;
    TempPub.Publish();
    float target = FanTargetFromTemp(tempC);
    UpdateFanFromTarget(target);
    
    led_r = 0.0f;
    led_g = 0.0f;
    led_b = target;
    ApplyLedPWM();
  } else {
    logMsg.Set("[WARN] MCP9801 read FAILED (still)");
    LogPub.Publish();
    led_r = 1.0f;
    led_g = 0.0f;
    led_b = 0.0f;
    ApplyLedPWM();

    UpdateFanFromTarget(1.0f);
  }
}


// #include "MicroROS.h"
// #include <Arduino.h>

// // #include <rcl/rcl.h>
// // #include <rclc/rclc.h>
// // #include <rclc/executor.h>
// #include <std_msgs/msg/int32.h>

// // #include <std_msgs/msg/int32_multi_array.h>

// #define NODE_NAME "drive"
// #define NAMESPACE ""
// // #define SUBSCRIPTION_NAME "arm_target_motor_positions"
// // #define PUBLISHER_NAME "get_arm_position"

// // rclc_executor_t executor;
// // rcl_subscription_t subscriber;
// // rcl_publisher_t publisher;

// uROS::Publisher p;

// void setup() {

//   // Configure LED pin
//   pinMode(13, OUTPUT);
//   tone(13, 1, 3000);

//   uROS::Init_MicroROS(NODE_NAME, NAMESPACE);

//   p.Init("drive_test");
// }

// int i = 0;
// std_msgs__msg__Int32 _out_msg;
// void loop() {
//   delay(1000);

//   _out_msg.data = i++;
//   p.PublishRaw(&_out_msg);
//   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
// }
