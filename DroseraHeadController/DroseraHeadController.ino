// ============================================================
// Drosera Head Controller — stepper-removed board revision
//
// Integrated features:
//  - SSD1306 OLED on GPIO8/GPIO9.
//  - Servo PCA9685 at 0x40; logical servos 0-7 -> PCA channels 8-15.
//  - Zero-based servo limits: /robot/servolimits/0 through /7.
//  - Eight-servo telemetry on /bot02/status/servos.
//  - LED PCA9685 at 0x60 with schematic channel mapping.
//  - Safe physical servo enable/disable and accurate applied-position state.
//  - Non-blocking safe-pose transition between DIRECT, EASE, and AUTO.
//  - Strict UDP OSC validation and bounded command durations.
//  - Complete discard of overlong Serial and Daisy UART lines.
//  - Runtime Wi-Fi, UDP, and mDNS recovery.
//  - Daisy UART heartbeat/link telemetry.
//  - Four DRV2605 haptics behind TCA9546A, including Purr arbitration.
// ============================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <OSCMessage.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPR121.h>
#include <Wire.h>
#include <UMS3.h>
#include <Adafruit_DRV2605.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <stdlib.h>
#include "Secrets.h"
#include "EasingFunctions.h"
#include "AutoMode.h"
#include "Purr.h"

#define DEBUG_VERBOSE
// #define DEBUG_OSC_RAW

// ----------------------------
// Constants
// ----------------------------
static constexpr uint8_t BUTTON_PIN = 0;
static constexpr uint8_t PWM_ENABLE_PIN = 7;
static constexpr uint8_t I2C1_SDA = 8;
static constexpr uint8_t I2C1_SCL = 9;
static constexpr uint32_t I2C_CLOCK_HZ = 100000;

static constexpr uint8_t SCREEN_WIDTH = 128;
static constexpr uint8_t SCREEN_HEIGHT = 64;
static constexpr int8_t OLED_RESET = -1;
static constexpr uint8_t SCREEN_ADDRESS = 0x3C;

static constexpr uint8_t NUM_SERVOS = 8;
static constexpr int SERVO_PULSE_MIN_US = 500;
static constexpr int SERVO_PULSE_MAX_US = 2400;
static constexpr unsigned long MODE_TRANSITION_DURATION_MS = 2500;

static constexpr uint8_t NUM_LEDS = 12;
static constexpr bool LED_OUTPUT_INVERTED = false;

static constexpr uint8_t TCA9546A_ADDRESS = 0x70;
static constexpr uint8_t DRV2605_ADDRESS = 0x5A;
static constexpr uint8_t NUM_HAPTICS = 4;
static constexpr uint8_t PURR_HAPTIC_CHANNEL = 0;
static constexpr bool HAPTIC_MOTORS_ARE_LRA = false;
static constexpr unsigned long HAPTIC_ONESHOT_GUARD_MS = 1500;

static constexpr unsigned long MAX_COMMAND_DURATION_MS = 600000;
static constexpr unsigned long CONTROL_INTERVAL_MS = 10;
static constexpr unsigned long OLED_REFRESH_INTERVAL_MS = 500;
static constexpr unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000;
static constexpr size_t MAX_SERIAL_LINE_LENGTH = 255;
static constexpr size_t MAX_DAISY_LINE_LENGTH = 63;
static constexpr int MAX_OSC_PACKET_SIZE = 512;

// ----------------------------
// Motion state
// ----------------------------
enum MotionMode { MODE_DIRECT, MODE_EASE, MODE_AUTO };
MotionMode currentMode = MODE_DIRECT;
EasingType currentEasing = EASE_INOUT_SINE;

struct EaseMove {
  float startPan = 90.0f;
  float startTilt = 90.0f;
  float targetPan = 90.0f;
  float targetTilt = 90.0f;
  unsigned long startTime = 0;
  unsigned long duration = 1000;
  bool active = false;
};

EaseMove currentEaseMove;
bool easeDoneNotified = false;

// ----------------------------
// Hardware objects
// ----------------------------
UMS3 ums3;
Adafruit_MPR121 cap;
Adafruit_DRV2605 drv;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_PWMServoDriver servoDriver(0x40);
Adafruit_PWMServoDriver ledDriver(0x60);

// ----------------------------
// General UI state
// ----------------------------
bool lastButtonState = HIGH;
unsigned long lastButtonChange = 0;
int testColorIndex = 0;
const uint32_t testColors[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00};

bool mpr121Found = false;
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
int displayMode = 0;
unsigned long lastOSCActivity = 0;
unsigned long lastDaisyTX = 0;

// ----------------------------
// Wi-Fi / OSC
// ----------------------------
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

WiFiUDP udp;

const char* robotName = ROBOT_NAME;
String robotNameStr(robotName);

const uint16_t localPort = LOCAL_PORT;
//IPAddress maxIP(10, 0, 0, 178);
const uint16_t maxPort = MAX_PORT;

bool udpRunning = false;
bool mdnsRunning = false;
unsigned long lastWiFiReconnectAttempt = 0;

// ----------------------------
// Servo state
// ----------------------------
static constexpr uint8_t SERVO_PCA_CHANNEL[NUM_SERVOS] = {
  8, 9, 10, 11, 12, 13, 14, 15
};

static const char* const SERVO_LIMIT_ADDRESS[NUM_SERVOS] = {
  "/robot/servolimits/0",
  "/robot/servolimits/1",
  "/robot/servolimits/2",
  "/robot/servolimits/3",
  "/robot/servolimits/4",
  "/robot/servolimits/5",
  "/robot/servolimits/6",
  "/robot/servolimits/7"
};

float servoPos[NUM_SERVOS];
float servoMin[NUM_SERVOS];
float servoMax[NUM_SERVOS];
float latestServoTarget[NUM_SERVOS];
float servoSafePosition[NUM_SERVOS] = {
  90.0f, 90.0f, 90.0f, 90.0f,
  90.0f, 90.0f, 90.0f, 90.0f
};

int activeServoCount = 0;
float servoSpeed = 1.0f;

struct ModeTransition {
  bool active = false;
  MotionMode pendingMode = MODE_DIRECT;
  unsigned long startTime = 0;
  unsigned long duration = MODE_TRANSITION_DURATION_MS;
  uint8_t servoCount = 0;
  float startPosition[NUM_SERVOS];
};

ModeTransition modeTransition;

// ----------------------------
// LED state
// ----------------------------
static constexpr uint8_t LED_PCA_CHANNEL[NUM_LEDS] = {
  7, 6, 5, 4, 3, 2, 1, 0,
  8, 9, 10, 11
};

struct LedFade {
  float startVal = 0.0f;
  float targetVal = 0.0f;
  unsigned long startTime = 0;
  unsigned long duration = 0;
  bool active = false;
};

float ledValue[NUM_LEDS];
LedFade ledFades[NUM_LEDS];
EasingType currentLedEasing = EASE_INOUT_SINE;

// ----------------------------
// Haptic state
// ----------------------------
bool hapticFound[NUM_HAPTICS] = {false, false, false, false};
uint8_t hapticHealthMask = 0;
uint8_t selectedHapticMask = 0xFF;
unsigned long purrSuspendedUntil = 0;

// ----------------------------
// System state
// ----------------------------
struct SystemState {
  bool wifi_ok = false;
  bool display_ok = false;
  bool mpr121_ok = false;
  bool drv2605_ok = false;
  bool daisy_ok = false;
};
SystemState systemState;

int lastSeq = -1;
unsigned long lastPacketTime = 0;
unsigned long lastControl = 0;

// ----------------------------
// Daisy UART
// ----------------------------
HardwareSerial DaisySerial(2);
static constexpr int DAISY_RX_PIN = 18;
static constexpr int DAISY_TX_PIN = 17;
static constexpr uint32_t DAISY_BAUD = 115200;

static constexpr int DAISY_ID_PING = 99;
static constexpr int DAISY_ID_CPU_LOAD = 50;
static constexpr int DAISY_ID_READY = 51;
static constexpr int DAISY_ID_HEARTBEAT = 52;
static constexpr int DAISY_ID_ERROR = 53;
static constexpr int DAISY_ID_AUDIO = 54;
static constexpr unsigned long DAISY_PING_INTERVAL_MS = 1000;
static constexpr unsigned long DAISY_LINK_TIMEOUT_MS = 2500;

unsigned long lastDaisyRX = 0;
unsigned long lastDaisyPing = 0;
uint32_t daisyRxPackets = 0;
uint32_t daisyParseErrors = 0;
int daisyCpuLoad = 0;
unsigned long lastCpuUpdate = 0;
int daisyLastError = 0;
int daisyAudioLevel = 0;

// ----------------------------
// Telemetry timers
// ----------------------------
unsigned long lastSys = 0;
unsigned long lastMem = 0;
unsigned long lastNet = 0;
unsigned long lastServo = 0;
unsigned long lastDaisy = 0;
unsigned long lastHaptic = 0;
unsigned long lastOLED = 0;

// ----------------------------
// Forward declarations
// ----------------------------
void updateOLED();
void sendOSC(OSCMessage &msg);
void setServoAngle(uint8_t channel, float angle);
void setActiveServoCount(int requestedCount);
void processSerialOSC(String message);
void processHapticCommand(String args);
void requestEaseMove(float pan, float tilt, unsigned long durationMs);
void sendTouchStatus();
void sendDaisyStatus();
void sendHapticStatus();
void sendModeStatus(const char* event);
void updateDaisyLink();
void readDaisySerial();
void motorControlLoop();
void telemetryLoop();
void handleOSC();
void handleSerialOSC();
void handleTouch();
void handleButton();

// ----------------------------
// General parsing helpers
// ----------------------------
float getOSCNumber(OSCMessage &msg, int index) {
  if (msg.isFloat(index)) return msg.getFloat(index);
  if (msg.isInt(index)) return (float)msg.getInt(index);
  return 0.0f;
}

bool oscArgumentIsNumber(OSCMessage &msg, int index) {
  return index >= 0 && index < msg.size()
      && (msg.isInt(index) || msg.isFloat(index));
}

bool requireOSCArguments(OSCMessage &msg, int count, const char* command) {
  if (msg.size() != count) {
    Serial.printf("%s rejected: expected %d arguments, received %d\n",
                  command, count, msg.size());
    return false;
  }
  return true;
}

bool requireOSCNumericArguments(OSCMessage &msg,
                                int count,
                                const char* command) {
  if (!requireOSCArguments(msg, count, command)) return false;
  for (int i = 0; i < count; ++i) {
    if (!oscArgumentIsNumber(msg, i)) {
      Serial.printf("%s rejected: argument %d is not numeric\n", command, i);
      return false;
    }
  }
  return true;
}

bool parseOneIntStrict(const String &args, int &value) {
  char extra = '\0';
  return sscanf(args.c_str(), " %d %c", &value, &extra) == 1;
}

bool parseOneFloatStrict(const String &args, float &value) {
  char extra = '\0';
  return sscanf(args.c_str(), " %f %c", &value, &extra) == 1;
}

bool parseTwoFloatsStrict(const String &args, float &first, float &second) {
  char extra = '\0';
  return sscanf(args.c_str(), " %f %f %c", &first, &second, &extra) == 2;
}

bool parseTwoIntsStrict(const String &args, int &first, int &second) {
  char extra = '\0';
  return sscanf(args.c_str(), " %d %d %c", &first, &second, &extra) == 2;
}

bool parseTwoFloatsAndDurationStrict(const String &args,
                                     float &first,
                                     float &second,
                                     long &durationMs) {
  char extra = '\0';
  return sscanf(args.c_str(), " %f %f %ld %c",
                &first, &second, &durationMs, &extra) == 3;
}

bool parseChannelTargetDurationStrict(const String &args,
                                      int &channel,
                                      float &target,
                                      long &durationMs) {
  char extra = '\0';
  return sscanf(args.c_str(), " %d %f %ld %c",
                &channel, &target, &durationMs, &extra) == 3;
}

bool validateDuration(long durationMs,
                      unsigned long &validated,
                      const char* command) {
  if (durationMs < 0 || durationMs > (long)MAX_COMMAND_DURATION_MS) {
    Serial.printf("%s rejected: duration must be 0-%lums\n",
                  command, MAX_COMMAND_DURATION_MS);
    return false;
  }
  validated = (unsigned long)durationMs;
  return true;
}

bool parseEightFloatsStrict(const String &args, float values[NUM_SERVOS]) {
  char extra = '\0';
  return sscanf(args.c_str(),
                " %f %f %f %f %f %f %f %f %c",
                &values[0], &values[1], &values[2], &values[3],
                &values[4], &values[5], &values[6], &values[7],
                &extra) == NUM_SERVOS;
}

// ----------------------------
// Servo-limit helpers
// ----------------------------
int servoLimitChannelForMessage(OSCMessage &msg) {
  for (uint8_t channel = 0; channel < NUM_SERVOS; ++channel) {
    if (msg.fullMatch(SERVO_LIMIT_ADDRESS[channel])) return channel;
  }
  return -1;
}

bool parseServoLimitAddress(const String &address, uint8_t &channel) {
  static const char prefix[] = "/robot/servolimits/";
  if (!address.startsWith(prefix)) return false;

  const String suffix = address.substring(sizeof(prefix) - 1);
  if (suffix.length() != 1 || suffix[0] < '0' || suffix[0] > '7') {
    return false;
  }

  channel = (uint8_t)(suffix[0] - '0');
  return channel < NUM_SERVOS;
}

void setServoLimits(uint8_t channel, float minimum, float maximum) {
  if (channel >= NUM_SERVOS) return;

  minimum = constrain(minimum, 0.0f, 180.0f);
  maximum = constrain(maximum, 0.0f, 180.0f);

  if (minimum > maximum) {
    const float temporary = minimum;
    minimum = maximum;
    maximum = temporary;
  }

  servoMin[channel] = minimum;
  servoMax[channel] = maximum;
  latestServoTarget[channel] = constrain(latestServoTarget[channel],
                                         minimum, maximum);

  Serial.printf("Servo %u limits -> %.1f-%.1f degrees\n",
                channel, servoMin[channel], servoMax[channel]);
}

// ----------------------------
// I2C helpers
// ----------------------------
bool i2cAddressResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void scanI2C() {
  Serial.println("Scanning I2C1 on SDA=GPIO8, SCL=GPIO9...");
  int found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    if (i2cAddressResponds(address)) {
      Serial.printf("  Found I2C device at 0x%02X\n", address);
      ++found;
    }
  }
  if (found == 0) Serial.println("  No I2C devices found");
}

bool readI2CRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool writeI2CRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool disablePcaAllCall(uint8_t address) {
  uint8_t mode1 = 0;
  if (!readI2CRegister(address, 0x00, mode1)) {
    Serial.printf("PCA9685 0x%02X: unable to read MODE1\n", address);
    return false;
  }

  const uint8_t updated = mode1 & (uint8_t)~0x01;
  if (!writeI2CRegister(address, 0x00, updated)) {
    Serial.printf("PCA9685 0x%02X: unable to clear ALLCALL\n", address);
    return false;
  }

  uint8_t verify = 0;
  const bool ok = readI2CRegister(address, 0x00, verify)
               && ((verify & 0x01) == 0);

  Serial.printf("PCA9685 0x%02X MODE1: 0x%02X -> 0x%02X; ALLCALL %s\n",
                address, mode1, verify, ok ? "disabled" : "FAILED");
  return ok;
}

// ----------------------------
// Haptic helpers
// ----------------------------
bool selectHapticChannel(uint8_t channel) {
  if (channel >= NUM_HAPTICS) return false;

  const uint8_t mask = (uint8_t)(1U << channel);
  Wire.beginTransmission(TCA9546A_ADDRESS);
  Wire.write(mask);
  const bool ok = Wire.endTransmission() == 0;
  if (ok) selectedHapticMask = mask;
  delayMicroseconds(500);
  return ok;
}

void disableAllHapticChannels() {
  Wire.beginTransmission(TCA9546A_ADDRESS);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() == 0) selectedHapticMask = 0;
  delayMicroseconds(500);
}

bool initializeHapticChannel(uint8_t channel) {
  if (!selectHapticChannel(channel)) {
    Serial.printf("Haptic %u: mux selection failed\n", channel);
    return false;
  }

  Wire.beginTransmission(DRV2605_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.printf("Haptic %u: no DRV2605 at 0x5A\n", channel);
    return false;
  }

  if (!drv.begin(&Wire)) {
    Serial.printf("Haptic %u: DRV2605 initialization failed\n", channel);
    return false;
  }

  drv.selectLibrary(HAPTIC_MOTORS_ARE_LRA ? 6 : 1);
  if (HAPTIC_MOTORS_ARE_LRA) drv.useLRA();
  drv.setMode(DRV2605_MODE_INTTRIG);

  Serial.printf("Haptic %u initialized (%s)\n",
                channel, HAPTIC_MOTORS_ARE_LRA ? "LRA" : "ERM");
  return true;
}

void initializeHaptics() {
  hapticHealthMask = 0;
  disableAllHapticChannels();

  for (uint8_t channel = 0; channel < NUM_HAPTICS; ++channel) {
    hapticFound[channel] = initializeHapticChannel(channel);
    if (hapticFound[channel]) hapticHealthMask |= (uint8_t)(1U << channel);
  }

  systemState.drv2605_ok = hapticHealthMask != 0;
  Serial.printf("Haptic health mask: 0x%X (%u/4 found)\n",
                hapticHealthMask,
                __builtin_popcount((unsigned int)hapticHealthMask));

  if (hapticFound[PURR_HAPTIC_CHANNEL]
      && selectHapticChannel(PURR_HAPTIC_CHANNEL)) {
    purrInit(&drv);
  } else {
    disableAllHapticChannels();
    Serial.println("Purr disabled: designated haptic channel unavailable");
  }
}

bool purrIsSuspended() {
  return (int32_t)(millis() - purrSuspendedUntil) < 0;
}

void suspendPurrForOneShot() {
  purrSuspendedUntil = millis() + HAPTIC_ONESHOT_GUARD_MS;
}

// ----------------------------
// Network service management
// ----------------------------
void stopNetworkServices() {
  if (udpRunning) {
    udp.stop();
    udpRunning = false;
  }
  if (mdnsRunning) {
    MDNS.end();
    mdnsRunning = false;
  }
}

void startNetworkServices() {
  if (WiFi.status() != WL_CONNECTED) return;

  stopNetworkServices();

  if (udp.begin(localPort)) {
    udpRunning = true;
    Serial.printf("OSC UDP listening on port %u\n", localPort);
  } else {
    Serial.printf("OSC UDP failed to start on port %u\n", localPort);
  }

  if (MDNS.begin(robotName)) {
    MDNS.addService("osc", "udp", localPort);
    mdnsRunning = true;
    Serial.printf("Robot available at %s.local\n", robotName);
  } else {
    Serial.println("mDNS failed");
  }
}

void updateWiFiLink() {
  const unsigned long now = millis();
  const bool connected = WiFi.status() == WL_CONNECTED;

  if (connected) {
    if (!systemState.wifi_ok) {
      systemState.wifi_ok = true;
      Serial.print("WiFi reconnected: ");
      Serial.println(WiFi.localIP());
      startNetworkServices();
      if (systemState.display_ok) updateOLED();
    }
    return;
  }

  if (systemState.wifi_ok) {
    systemState.wifi_ok = false;
    Serial.println("WiFi disconnected");
    stopNetworkServices();
    if (systemState.display_ok) updateOLED();
  }

  if (now - lastWiFiReconnectAttempt >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWiFiReconnectAttempt = now;
    Serial.println("Attempting WiFi reconnect...");
    WiFi.reconnect();
  }
}

// ----------------------------
// Setup
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Keep all PCA9685 outputs disabled until both chips are configured.
  pinMode(PWM_ENABLE_PIN, OUTPUT);
  digitalWrite(PWM_ENABLE_PIN, HIGH);

  ums3.begin();
  delay(100);
  ums3.setPixelColor(0xFF0000);

  if (!Wire.begin(I2C1_SDA, I2C1_SCL, I2C_CLOCK_HZ)) {
    Serial.println("Wire.begin() failed");
    while (true) delay(1000);
  }

  scanI2C();

  if (!display.begin(SSD1306_SWITCHCAPVCC,
                     SCREEN_ADDRESS,
                     true,
                     false)) {
    Serial.println(F("SSD1306 initialization failed"));
  } else {
    systemState.display_ok = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
    Serial.println("OLED initialized at 0x3C");
  }

  WiFi.begin(ssid, password);
  const unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED
         && millis() - wifiStart < 10000UL) {
    delay(500);
    Serial.print('.');
    if (systemState.display_ok) {
      display.print('.');
      display.display();
    }
  }

  systemState.wifi_ok = WiFi.status() == WL_CONNECTED;
  if (systemState.wifi_ok) {
    Serial.println("\nWiFi connected");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection timeout - continuing without WiFi");
  }
  WiFi.setSleep(false);

  if (systemState.wifi_ok) {
    startNetworkServices();
  } else {
    Serial.println("Network services deferred until WiFi connects");
    lastWiFiReconnectAttempt = millis();
  }

  if (!cap.begin(0x5C, &Wire)) {
    Serial.println("MPR121 not found");
    ums3.setPixelColor(0xFF0000);
  } else {
    Serial.println("MPR121 found");
    cap.setAutoconfig(true);
    cap.setThresholds(12, 6);
    delay(500);
    mpr121Found = true;
    systemState.mpr121_ok = true;
    ums3.setPixelColor(0x00FF00);
  }

  DaisySerial.begin(DAISY_BAUD, SERIAL_8N1,
                    DAISY_RX_PIN, DAISY_TX_PIN);

  for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
    servoMin[i] = 0.0f;
    servoMax[i] = 180.0f;
    servoPos[i] = 90.0f;
    latestServoTarget[i] = 90.0f;
  }

  servoDriver.begin();
  servoDriver.setPWMFreq(50);
  delay(10);
  for (uint8_t channel = 0; channel < 16; ++channel) {
    servoDriver.setPWM(channel, 0, 4096);
  }
  Serial.println("Servo PCA9685 initialized at 0x40");

  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    ledValue[i] = 0.0f;
    ledFades[i].active = false;
  }

  ledDriver.begin();
  ledDriver.setPWMFreq(1000);
  delay(10);
  for (uint8_t channel = 0; channel < 16; ++channel) {
    ledDriver.setPWM(channel, 0, 4096);
  }
  Serial.println("LED PCA9685 initialized at 0x60");

  const bool servoAllCallOff = disablePcaAllCall(0x40);
  const bool ledAllCallOff = disablePcaAllCall(0x60);
  if (!servoAllCallOff || !ledAllCallOff) {
    Serial.println("WARNING: PCA9685 ALLCALL conflict was not fully cleared");
  }

  // Enable the configured PCA9685 outputs. Individual servo outputs remain
  // full-off until activeServoCount is increased.
  digitalWrite(PWM_ENABLE_PIN, LOW);
  Serial.println("PWM outputs enabled on GPIO7");

  initializeHaptics();

  if (systemState.display_ok) updateOLED();
}

// ----------------------------
// Main loop
// ----------------------------
void loop() {
  updateWiFiLink();
  handleOSC();
  handleSerialOSC();
  handleTouch();
  handleButton();
  readDaisySerial();
  updateDaisyLink();
  motorControlLoop();
  telemetryLoop();
}

// ----------------------------
// Safe mode switching
// ----------------------------
const char* motionModeName(MotionMode mode) {
  switch (mode) {
    case MODE_DIRECT: return "DIRECT";
    case MODE_EASE: return "EASE";
    case MODE_AUTO: return "AUTO";
    default: return "UNKNOWN";
  }
}

const char* motionModeShortName(MotionMode mode) {
  switch (mode) {
    case MODE_DIRECT: return "DIR";
    case MODE_EASE: return "EAS";
    case MODE_AUTO: return "AUT";
    default: return "???";
  }
}

int modeTransitionProgressPercent() {
  if (!modeTransition.active) return 100;
  const unsigned long elapsed = millis() - modeTransition.startTime;
  if (elapsed >= modeTransition.duration) return 100;
  return (int)((elapsed * 100UL) / modeTransition.duration);
}

void sendModeStatus(const char* event) {
  String address = String('/') + robotNameStr + "/status/mode";
  OSCMessage msg(address.c_str());
  msg.add(event);
  msg.add((int32_t)currentMode);
  msg.add((int32_t)modeTransition.pendingMode);
  msg.add((int32_t)modeTransition.active);
  msg.add((int32_t)modeTransitionProgressPercent());
  msg.add((int32_t)modeTransition.servoCount);
  sendOSC(msg);
}

void finishModeTransition() {
  const MotionMode completedMode = modeTransition.pendingMode;

  for (uint8_t ch = 0; ch < modeTransition.servoCount; ++ch) {
    const float safe = constrain(servoSafePosition[ch],
                                 servoMin[ch], servoMax[ch]);
    setServoAngle(ch, safe);
  }

  modeTransition.active = false;
  currentMode = completedMode;
  currentEaseMove.active = false;
  easeDoneNotified = false;

  if (currentMode == MODE_DIRECT) {
    const uint8_t count = (uint8_t)constrain(activeServoCount,
                                              0, (int)NUM_SERVOS);
    for (uint8_t ch = 0; ch < count; ++ch) {
      latestServoTarget[ch] = servoPos[ch];
    }
  } else if (currentMode == MODE_AUTO) {
    autoModeInit();
    autoModeReset();
  }

  Serial.printf("Mode transition complete -> %s\n",
                motionModeName(currentMode));
  sendModeStatus("complete");
}

void beginModeTransition(MotionMode newMode) {
  modeTransition.active = true;
  modeTransition.pendingMode = newMode;
  modeTransition.startTime = millis();
  modeTransition.duration = MODE_TRANSITION_DURATION_MS;
  modeTransition.servoCount =
      (uint8_t)constrain(activeServoCount, 0, (int)NUM_SERVOS);

  for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) {
    modeTransition.startPosition[ch] = servoPos[ch];
  }

  currentEaseMove.active = false;
  easeDoneNotified = false;

  Serial.printf("Mode transition: %s -> CENTER -> %s (%lums, %u servos)\n",
                motionModeName(currentMode),
                motionModeName(modeTransition.pendingMode),
                modeTransition.duration,
                modeTransition.servoCount);
  sendModeStatus("centering");

  if (modeTransition.servoCount == 0) finishModeTransition();
}

void updateModeTransition() {
  if (!modeTransition.active) return;

  const unsigned long elapsed = millis() - modeTransition.startTime;
  if (elapsed >= modeTransition.duration) {
    finishModeTransition();
    return;
  }

  const float t = (float)elapsed / (float)modeTransition.duration;
  const float easedT = applyEasing(t, EASE_INOUT_SINE);

  for (uint8_t ch = 0; ch < modeTransition.servoCount; ++ch) {
    const float safe = constrain(servoSafePosition[ch],
                                 servoMin[ch], servoMax[ch]);
    const float angle = modeTransition.startPosition[ch]
                      + (safe - modeTransition.startPosition[ch]) * easedT;
    setServoAngle(ch, angle);
  }
}

void setMode(int requestedMode) {
  requestedMode = constrain(requestedMode, 0, 2);
  const MotionMode newMode = (MotionMode)requestedMode;

  if (modeTransition.active) {
    if (newMode != modeTransition.pendingMode) {
      modeTransition.pendingMode = newMode;
      Serial.printf("Mode transition destination -> %s\n",
                    motionModeName(newMode));
      sendModeStatus("retargeted");
    }
    return;
  }

  if (newMode != currentMode) beginModeTransition(newMode);
}

// ----------------------------
// Easing / EASE moves
// ----------------------------
const char* easingName(EasingType type) {
  switch (type) {
    case EASE_LINEAR: return "LINEAR";
    case EASE_IN_CUBIC: return "IN_CUBIC";
    case EASE_OUT_CUBIC: return "OUT_CUBIC";
    case EASE_INOUT_CUBIC: return "INOUT_CUBIC";
    case EASE_INOUT_SINE: return "INOUT_SINE";
    case EASE_INOUT_QUAD: return "INOUT_QUAD";
    case EASE_OUT_ELASTIC: return "OUT_ELASTIC";
    case EASE_INOUT_EXPO: return "INOUT_EXPO";
    default: return "UNKNOWN";
  }
}

void setEasingMode(int value) {
  value = constrain(value, 0, 7);
  currentEasing = (EasingType)value;
  Serial.printf("Easing -> %s\n", easingName(currentEasing));
}

void setLedEasingMode(int value) {
  value = constrain(value, 0, 7);
  currentLedEasing = (EasingType)value;
  Serial.printf("LED Easing -> %s\n", easingName(currentLedEasing));
}

void startEaseMove(float targetPan,
                   float targetTilt,
                   unsigned long durationMs) {
  currentEaseMove.startPan = servoPos[0];
  currentEaseMove.startTilt = servoPos[1];
  currentEaseMove.targetPan = constrain(targetPan,
                                         servoMin[0], servoMax[0]);
  currentEaseMove.targetTilt = constrain(targetTilt,
                                          servoMin[1], servoMax[1]);
  currentEaseMove.startTime = millis();
  currentEaseMove.duration = max(1UL, durationMs);
  currentEaseMove.active = true;
  easeDoneNotified = false;

  Serial.printf("EaseMove: pan %.1f->%.1f tilt %.1f->%.1f dur %lums\n",
                currentEaseMove.startPan, currentEaseMove.targetPan,
                currentEaseMove.startTilt, currentEaseMove.targetTilt,
                currentEaseMove.duration);
}

void requestEaseMove(float targetPan,
                     float targetTilt,
                     unsigned long durationMs) {
  if (modeTransition.active) {
    if (modeTransition.pendingMode != MODE_EASE) setMode(MODE_EASE);
    Serial.println("Ease command deferred: resend after centering completes");
    return;
  }

  if (currentMode != MODE_EASE) {
    setMode(MODE_EASE);
    Serial.println("Ease command deferred: switching to EASE; resend after centering");
    return;
  }

  startEaseMove(targetPan, targetTilt, durationMs);
}

void sendEaseDone() {
  String address = String('/') + robotNameStr + "/status/ease";
  OSCMessage msg(address.c_str());
  msg.add("done");
  msg.add((int32_t)lroundf(servoPos[0]));
  msg.add((int32_t)lroundf(servoPos[1]));
  sendOSC(msg);
  Serial.println("EaseMove: done");
}

// ----------------------------
// Servo hardware and count safety
// ----------------------------
void setServoAngle(uint8_t channel, float angle) {
  if (channel >= NUM_SERVOS || channel >= activeServoCount) return;

  angle = constrain(angle, servoMin[channel], servoMax[channel]);
  const long pulseUs = map((long)lroundf(angle), 0L, 180L,
                           (long)SERVO_PULSE_MIN_US,
                           (long)SERVO_PULSE_MAX_US);
  const uint16_t ticks =
      (uint16_t)((pulseUs * 4096L) / 20000L);

  servoDriver.setPWM(SERVO_PCA_CHANNEL[channel], 0, ticks);
  servoPos[channel] = angle;
}

void writeServo(uint8_t channel, float angle) {
  setServoAngle(channel, angle);
}

void setActiveServoCount(int requestedCount) {
  const int newCount = constrain(requestedCount, 0, (int)NUM_SERVOS);
  const int oldCount = activeServoCount;
  if (newCount == oldCount) return;

  if (modeTransition.active && newCount > oldCount) {
    Serial.printf("Servo count increase rejected during centering: %d -> %d\n",
                  oldCount, newCount);
    return;
  }

  if (newCount < oldCount) {
    for (int ch = newCount; ch < oldCount; ++ch) {
      servoDriver.setPWM(SERVO_PCA_CHANNEL[ch], 0, 4096);
    }

    activeServoCount = newCount;
    if (modeTransition.active
        && modeTransition.servoCount > (uint8_t)newCount) {
      modeTransition.servoCount = (uint8_t)newCount;
    }

    Serial.printf("activeServoCount -> %d; disabled logical channels %d-%d\n",
                  activeServoCount, newCount, oldCount - 1);
    return;
  }

  activeServoCount = newCount;
  for (int ch = oldCount; ch < newCount; ++ch) {
    const float safe = constrain(servoSafePosition[ch],
                                 servoMin[ch], servoMax[ch]);
    latestServoTarget[ch] = safe;
    setServoAngle((uint8_t)ch, safe);
  }

  Serial.printf("activeServoCount -> %d; enabled logical channels %d-%d at safe pose\n",
                activeServoCount, oldCount, newCount - 1);
}

// ----------------------------
// LED hardware
// ----------------------------
void writeLedOutput(uint8_t channel, float value) {
  if (channel >= NUM_LEDS) return;
  const uint16_t pwm =
      (uint16_t)constrain(lroundf(value), 0L, 4095L);
  ledDriver.setPin(LED_PCA_CHANNEL[channel], pwm, LED_OUTPUT_INVERTED);
}

void setLedInstant(uint8_t channel, float value) {
  if (channel >= NUM_LEDS) {
    Serial.printf("LED: invalid channel %u\n", channel);
    return;
  }

  value = constrain(value, 0.0f, 4095.0f);
  ledFades[channel].active = false;
  ledValue[channel] = value;
  writeLedOutput(channel, value);

#ifdef DEBUG_VERBOSE
  Serial.printf("LED: logical %u -> PCA %u -> %.0f\n",
                channel, LED_PCA_CHANNEL[channel], value);
#endif
}

void startLedFade(uint8_t channel,
                  float target,
                  unsigned long durationMs) {
  if (channel >= NUM_LEDS) return;

  target = constrain(target, 0.0f, 4095.0f);
  if (durationMs == 0) {
    setLedInstant(channel, target);
    return;
  }

  ledFades[channel].startVal = ledValue[channel];
  ledFades[channel].targetVal = target;
  ledFades[channel].startTime = millis();
  ledFades[channel].duration = durationMs;
  ledFades[channel].active = true;
}

void updateLedFades() {
  const unsigned long now = millis();
  for (uint8_t channel = 0; channel < NUM_LEDS; ++channel) {
    if (!ledFades[channel].active) continue;

    const unsigned long elapsed = now - ledFades[channel].startTime;
    if (elapsed >= ledFades[channel].duration) {
      ledValue[channel] = ledFades[channel].targetVal;
      writeLedOutput(channel, ledValue[channel]);
      ledFades[channel].active = false;
      continue;
    }

    const float t = (float)elapsed / (float)ledFades[channel].duration;
    const float easedT = applyEasing(t, currentLedEasing);
    ledValue[channel] = ledFades[channel].startVal
        + (ledFades[channel].targetVal - ledFades[channel].startVal)
        * easedT;
    writeLedOutput(channel, ledValue[channel]);
  }
}

// ----------------------------
// Serial OSC
// ----------------------------
void handleSerialOSC() {
  static String line;
  static bool overflow = false;

  while (Serial.available() > 0) {
    const char character = (char)Serial.read();

    if (character == '\n' || character == '\r') {
      if (!overflow && line.length() > 0) processSerialOSC(line);
      line = "";
      overflow = false;
      continue;
    }

    if (overflow) continue;

    if (line.length() < MAX_SERIAL_LINE_LENGTH) {
      line += character;
    } else {
      line = "";
      overflow = true;
      Serial.println("Serial OSC line rejected: too long");
    }
  }
}

void processSerialOSC(String message) {
  message.trim();
  if (message.length() == 0) return;

  Serial.print("Serial OSC: ");
  Serial.println(message);

  const int firstSpace = message.indexOf(' ');
  String address = message;
  String arguments;
  if (firstSpace >= 0) {
    address = message.substring(0, firstSpace);
    arguments = message.substring(firstSpace + 1);
    arguments.trim();
  }

  lastOSCActivity = millis();

  uint8_t servoLimitChannel = 0;
  const bool isServoLimitAddress =
      parseServoLimitAddress(address, servoLimitChannel);

  if (address == "/robot/servos") {
    float values[NUM_SERVOS];
    if (!parseEightFloatsStrict(arguments, values)) {
      Serial.println("/robot/servos rejected: expected exactly 8 numbers");
      return;
    }
    for (uint8_t channel = 0; channel < NUM_SERVOS; ++channel) {
      latestServoTarget[channel] = constrain(values[channel],
                                              servoMin[channel],
                                              servoMax[channel]);
    }
  } else if (address == "/robot/servos/count") {
    int count = 0;
    if (!parseOneIntStrict(arguments, count)) {
      Serial.println("Servo count rejected: expected one integer");
      return;
    }
    setActiveServoCount(count);
  } else if (address == "/robot/led") {
    int channel = 0;
    float value = 0.0f;
    char extra = '\0';
    if (sscanf(arguments.c_str(), " %d %f %c",
               &channel, &value, &extra) != 2) {
      Serial.println("/robot/led rejected: expected channel level");
      return;
    }
    if (channel < 0 || channel >= NUM_LEDS) {
      Serial.println("/robot/led rejected: invalid channel");
      return;
    }
    setLedInstant((uint8_t)channel, value);
  } else if (address == "/robot/led/fade") {
    int channel = 0;
    float target = 0.0f;
    long duration = 0;
    unsigned long validated = 0;
    if (!parseChannelTargetDurationStrict(arguments,
                                           channel, target, duration)
        || channel < 0 || channel >= NUM_LEDS
        || !validateDuration(duration, validated, "/robot/led/fade")) {
      Serial.println("/robot/led/fade rejected");
      return;
    }
    startLedFade((uint8_t)channel, target, validated);
  } else if (address == "/robot/led/fadeall") {
    float target = 0.0f;
    long duration = 0;
    char extra = '\0';
    unsigned long validated = 0;
    if (sscanf(arguments.c_str(), " %f %ld %c",
               &target, &duration, &extra) != 2
        || !validateDuration(duration, validated, "/robot/led/fadeall")) {
      Serial.println("/robot/led/fadeall rejected");
      return;
    }
    for (uint8_t channel = 0; channel < NUM_LEDS; ++channel) {
      startLedFade(channel, target, validated);
    }
  } else if (address == "/robot/led/easemode") {
    int value = 0;
    if (!parseOneIntStrict(arguments, value)) return;
    setLedEasingMode(value);
  } else if (address == "/haptic") {
    processHapticCommand(arguments);
  } else if (address == "/robot/purr/rate") {
    float value = 0.0f;
    if (!parseOneFloatStrict(arguments, value)) return;
    purrSetRootRate(value);
  } else if (address == "/robot/purr/intensity") {
    float value = 0.0f;
    if (!parseOneFloatStrict(arguments, value)) return;
    purrSetRootIntensity(value);
  } else if (address == "/robot/display") {
    int value = 0;
    if (!parseOneIntStrict(arguments, value)) return;
    displayMode = constrain(value, 0, 5);
    if (systemState.display_ok) updateOLED();
  } else if (address == "/speed") {
    float speed = 0.0f;
    if (!parseOneFloatStrict(arguments, speed)) return;
    servoSpeed = max(0.1f, speed / 100.0f);
  } else if (address == "/robot/mode") {
    int mode = 0;
    if (!parseOneIntStrict(arguments, mode)) return;
    setMode(mode);
  } else if (address == "/robot/ease") {
    float pan = 0.0f;
    float tilt = 0.0f;
    long duration = 0;
    unsigned long validated = 0;
    if (!parseTwoFloatsAndDurationStrict(arguments, pan, tilt, duration)
        || !validateDuration(duration, validated, "/robot/ease")) {
      Serial.println("/robot/ease rejected");
      return;
    }
    requestEaseMove(pan, tilt, validated);
  } else if (address == "/robot/easemode") {
    int value = 0;
    if (!parseOneIntStrict(arguments, value)) return;
    setEasingMode(value);
  } else if (address == "/robot/auto/speed") {
    float value = 0.0f;
    if (!parseOneFloatStrict(arguments, value)) return;
    autoSetSpeed(value);
  } else if (address == "/robot/auto/range") {
    float value = 0.0f;
    if (!parseOneFloatStrict(arguments, value)) return;
    autoSetRange(value);
  } else if (address == "/robot/auto/warp") {
    float value = 0.0f;
    if (!parseOneFloatStrict(arguments, value)) return;
    autoSetWarp(value);
  } else if (address == "/robot/auto/center") {
    float pan = 0.0f;
    float tilt = 0.0f;
    if (!parseTwoFloatsStrict(arguments, pan, tilt)) return;
    autoSetCenter(pan, tilt);
  } else if (isServoLimitAddress) {
    float minimum = 0.0f;
    float maximum = 0.0f;
    if (!parseTwoFloatsStrict(arguments, minimum, maximum)) {
      Serial.printf("Servo %u limits rejected: expected minimum maximum\n",
                    servoLimitChannel);
      return;
    }
    setServoLimits(servoLimitChannel, minimum, maximum);
  } else if (address == "/daisy") {
    int parameter = 0;
    int value = 0;
    if (!parseTwoIntsStrict(arguments, parameter, value)) return;
    sendToDaisy(parameter, value);
  } else if (address == "/daisy/synth") {
    int parameter = 0;
    int value = 0;
    if (!parseTwoIntsStrict(arguments, parameter, value)) return;
    sendToDaisy(parameter + 10, value);
  } else {
    Serial.printf("Unknown Serial OSC address: %s\n", address.c_str());
  }
}

void processHapticCommand(String args) {
  args.trim();
  int first = 0;
  int second = 0;

  if (parseOneIntStrict(args, first)) {
    playHaptic(first);
    return;
  }

  if (!parseTwoIntsStrict(args, first, second)) {
    Serial.println("/haptic rejected: expected effect or channel effect");
    return;
  }

  if (first < 0 || first >= NUM_HAPTICS) {
    Serial.printf("Haptic: invalid channel %d\n", first);
    return;
  }
  playOneHaptic(first, second);
}

// ----------------------------
// UDP OSC
// ----------------------------
void handleOSC() {
  if (!udpRunning) return;

  const int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  if (packetSize > MAX_OSC_PACKET_SIZE) {
    while (udp.available()) udp.read();
    Serial.printf("OSC packet rejected: %d bytes exceeds limit\n", packetSize);
    return;
  }

  byte buffer[MAX_OSC_PACKET_SIZE];
  const int bytesRead = udp.read(buffer, packetSize);

  OSCMessage msg;
  for (int i = 0; i < bytesRead; ++i) msg.fill(buffer[i]);
  if (msg.hasError()) {
    Serial.println("OSC parse error");
    return;
  }

  lastOSCActivity = millis();
  const int servoLimitChannel = servoLimitChannelForMessage(msg);

  if (msg.fullMatch("/robot/servos")) {
    if (!requireOSCNumericArguments(msg, NUM_SERVOS, "/robot/servos")) return;
    for (uint8_t channel = 0; channel < NUM_SERVOS; ++channel) {
      latestServoTarget[channel] = constrain(getOSCNumber(msg, channel),
                                              servoMin[channel],
                                              servoMax[channel]);
    }
  } else if (msg.fullMatch("/robot/servos/count")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/servos/count")) return;
    setActiveServoCount((int)getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/robot/led")) {
    if (!requireOSCNumericArguments(msg, 2, "/robot/led")) return;
    const int channel = (int)getOSCNumber(msg, 0);
    if (channel < 0 || channel >= NUM_LEDS) return;
    setLedInstant((uint8_t)channel, getOSCNumber(msg, 1));
  } else if (msg.fullMatch("/robot/led/fade")) {
    if (!requireOSCNumericArguments(msg, 3, "/robot/led/fade")) return;
    const int channel = (int)getOSCNumber(msg, 0);
    const long duration = (long)getOSCNumber(msg, 2);
    unsigned long validated = 0;
    if (channel < 0 || channel >= NUM_LEDS
        || !validateDuration(duration, validated, "/robot/led/fade")) return;
    startLedFade((uint8_t)channel, getOSCNumber(msg, 1), validated);
  } else if (msg.fullMatch("/robot/led/fadeall")) {
    if (!requireOSCNumericArguments(msg, 2, "/robot/led/fadeall")) return;
    const long duration = (long)getOSCNumber(msg, 1);
    unsigned long validated = 0;
    if (!validateDuration(duration, validated, "/robot/led/fadeall")) return;
    for (uint8_t channel = 0; channel < NUM_LEDS; ++channel) {
      startLedFade(channel, getOSCNumber(msg, 0), validated);
    }
  } else if (msg.fullMatch("/robot/led/easemode")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/led/easemode")) return;
    setLedEasingMode((int)getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/robot/mode")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/mode")) return;
    setMode((int)getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/robot/ease")) {
    if (!requireOSCNumericArguments(msg, 3, "/robot/ease")) return;
    const long duration = (long)getOSCNumber(msg, 2);
    unsigned long validated = 0;
    if (!validateDuration(duration, validated, "/robot/ease")) return;
    requestEaseMove(getOSCNumber(msg, 0), getOSCNumber(msg, 1), validated);
  } else if (msg.fullMatch("/robot/easemode")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/easemode")) return;
    setEasingMode((int)getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/speed")) {
    if (!requireOSCNumericArguments(msg, 1, "/speed")) return;
    servoSpeed = max(0.1f, getOSCNumber(msg, 0) / 100.0f);
  } else if (msg.fullMatch("/robot/auto/speed")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/auto/speed")) return;
    autoSetSpeed(getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/robot/auto/range")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/auto/range")) return;
    autoSetRange(getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/robot/auto/warp")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/auto/warp")) return;
    autoSetWarp(getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/robot/auto/center")) {
    if (!requireOSCNumericArguments(msg, 2, "/robot/auto/center")) return;
    autoSetCenter(getOSCNumber(msg, 0), getOSCNumber(msg, 1));
  } else if (servoLimitChannel >= 0) {
    if (!requireOSCNumericArguments(msg, 2, SERVO_LIMIT_ADDRESS[servoLimitChannel])) {
      return;
    }
    setServoLimits((uint8_t)servoLimitChannel,
                   getOSCNumber(msg, 0),
                   getOSCNumber(msg, 1));
  } else if (msg.fullMatch("/daisy/synth")) {
    if (!requireOSCNumericArguments(msg, 2, "/daisy/synth")) return;
    sendToDaisy((int)getOSCNumber(msg, 0) + 10,
                (int)getOSCNumber(msg, 1));
  } else if (msg.fullMatch("/daisy")) {
    if (!requireOSCNumericArguments(msg, 2, "/daisy")) return;
    sendToDaisy((int)getOSCNumber(msg, 0),
                (int)getOSCNumber(msg, 1));
  } else if (msg.fullMatch("/haptic")) {
    if (msg.size() == 1 && oscArgumentIsNumber(msg, 0)) {
      playHaptic((int)getOSCNumber(msg, 0));
    } else if (msg.size() == 2
               && oscArgumentIsNumber(msg, 0)
               && oscArgumentIsNumber(msg, 1)) {
      playOneHaptic((int)getOSCNumber(msg, 0),
                    (int)getOSCNumber(msg, 1));
    } else {
      Serial.println("/haptic rejected: expected effect or channel effect");
    }
  } else if (msg.fullMatch("/robot/purr/rate")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/purr/rate")) return;
    purrSetRootRate(getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/robot/purr/intensity")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/purr/intensity")) return;
    purrSetRootIntensity(getOSCNumber(msg, 0));
  } else if (msg.fullMatch("/robot/display")) {
    if (!requireOSCNumericArguments(msg, 1, "/robot/display")) return;
    displayMode = constrain((int)getOSCNumber(msg, 0), 0, 5);
    if (systemState.display_ok) updateOLED();
  } else if (msg.fullMatch("/test")) {
    if (!requireOSCNumericArguments(msg, 1, "/test")) return;
    const int color = constrain((int)getOSCNumber(msg, 0), 0, 3);
    ums3.setPixelColor(testColors[color]);
  }
}

// ----------------------------
// Touch and button
// ----------------------------
void handleTouch() {
  static unsigned long lastRead = 0;
  if (!mpr121Found || millis() - lastRead < 50) return;
  lastRead = millis();

  currtouched = cap.touched();
  if (currtouched == lasttouched) return;

  for (uint8_t pad = 0; pad < 12; ++pad) {
    const bool current = currtouched & _BV(pad);
    const bool previous = lasttouched & _BV(pad);
#ifdef DEBUG_VERBOSE
    if (current && !previous) Serial.printf("TOUCH: %u\n", pad);
    if (!current && previous) Serial.printf("RELEASE: %u\n", pad);
#endif
  }

  sendTouchStatus();
  purrSetTouched(currtouched);

  static const uint32_t touchColors[8] = {
    0xFFFFFF, 0xFF0000, 0xFF7F00, 0xFFFF00,
    0x00FF00, 0x0000FF, 0x4B0082, 0x8B00FF
  };

  if (currtouched) {
    for (uint8_t pad = 0; pad < 12; ++pad) {
      if (currtouched & _BV(pad)) {
        ums3.setPixelColor(pad < 8 ? touchColors[pad] : 0xFFFFFF);
        break;
      }
    }
  } else {
    ums3.setPixelColor(0x00FF00);
  }

  lasttouched = currtouched;
}

void handleButton() {
  const bool currentState = digitalRead(BUTTON_PIN);
  const unsigned long now = millis();

  if (currentState != lastButtonState
      && now - lastButtonChange >= 30) {
    lastButtonChange = now;
    lastButtonState = currentState;

    if (currentState == LOW) {
      Serial.println("BUTTON PRESSED");
      OSCMessage msg("/test");
      msg.add((int32_t)testColorIndex);
      sendOSC(msg);
      testColorIndex = (testColorIndex + 1) % 4;
    }
  }
}

// ----------------------------
// Control loop
// ----------------------------
void motorControlLoop() {
  if (!purrIsSuspended()
      && hapticFound[PURR_HAPTIC_CHANNEL]
      && selectHapticChannel(PURR_HAPTIC_CHANNEL)) {
    purrUpdate();
  }

  updateLedFades();

  if (millis() - lastControl < CONTROL_INTERVAL_MS) return;
  lastControl = millis();

  if (modeTransition.active) {
    updateModeTransition();
    return;
  }

  switch (currentMode) {
    case MODE_DIRECT:
      for (uint8_t channel = 0;
           channel < (uint8_t)activeServoCount;
           ++channel) {
        writeServo(channel, latestServoTarget[channel]);
      }
      break;

    case MODE_EASE:
      if (currentEaseMove.active) {
        const unsigned long elapsed =
            millis() - currentEaseMove.startTime;

        if (elapsed >= currentEaseMove.duration) {
          setServoAngle(0, currentEaseMove.targetPan);
          setServoAngle(1, currentEaseMove.targetTilt);
          currentEaseMove.active = false;

          if (!easeDoneNotified) {
            sendEaseDone();
            easeDoneNotified = true;
          }
        } else {
          const float t =
              (float)elapsed / (float)currentEaseMove.duration;
          const float easedT = applyEasing(t, currentEasing);
          setServoAngle(0,
              currentEaseMove.startPan
              + (currentEaseMove.targetPan - currentEaseMove.startPan)
              * easedT);
          setServoAngle(1,
              currentEaseMove.startTilt
              + (currentEaseMove.targetTilt - currentEaseMove.startTilt)
              * easedT);
        }
      }
      break;

    case MODE_AUTO: {
      float autoServoPan = 90.0f;
      float autoServoTilt = 90.0f;

      autoModeUpdate(autoServoPan, autoServoTilt);
      setServoAngle(0, autoServoPan);
      setServoAngle(1, autoServoTilt);
      break;
    }
  }
}

// ----------------------------
// Telemetry
// ----------------------------
void telemetryLoop() {
  const unsigned long now = millis();

  if (now - lastSys >= 1000) {
    lastSys = now;
    sendSystemStatus();
  }
  if (now - lastMem >= 2000) {
    lastMem = now;
    sendMemoryStatus();
  }
  if (now - lastNet >= 500) {
    lastNet = now;
    sendNetworkStatus();
  }
  if (now - lastServo >= 100) {
    lastServo = now;
    sendServoStatus();
  }
  if (now - lastDaisy >= 200) {
    lastDaisy = now;
    sendDaisyStatus();
  }
  if (now - lastHaptic >= 1000) {
    lastHaptic = now;
    sendHapticStatus();
  }
  if (now - lastOLED >= OLED_REFRESH_INTERVAL_MS) {
    lastOLED = now;
    updateOLED();
  }
}

void sendSystemStatus() {
  String address = String('/') + robotNameStr + "/status/system";
  OSCMessage msg(address.c_str());
  msg.add((int32_t)(millis() / 1000));
  msg.add((float)temperatureRead());
  msg.add((int32_t)getCpuFrequencyMhz());
  sendOSC(msg);
}

void sendMemoryStatus() {
  String address = String('/') + robotNameStr + "/status/memory";
  OSCMessage msg(address.c_str());
  msg.add((int32_t)ESP.getFreeHeap());
  msg.add((int32_t)ESP.getMinFreeHeap());
  msg.add((int32_t)ESP.getMaxAllocHeap());
  sendOSC(msg);
}

void sendNetworkStatus() {
  String address = String('/') + robotNameStr + "/status/network";
  OSCMessage msg(address.c_str());
  msg.add((int32_t)(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0));
  msg.add((int32_t)lastSeq);
  sendOSC(msg);
}

void sendServoStatus() {
  String address = String('/') + robotNameStr + "/status/servos";
  OSCMessage msg(address.c_str());
  for (uint8_t channel = 0; channel < NUM_SERVOS; ++channel) {
    msg.add((int32_t)lroundf(servoPos[channel]));
  }
  sendOSC(msg);
}

void sendTouchStatus() {
  String address = String('/') + robotNameStr + "/touch";
  OSCMessage msg(address.c_str());
  msg.add((int32_t)currtouched);
  sendOSC(msg);
}

void sendDaisyStatus() {
  String address = String('/') + robotNameStr + "/status/daisy";
  OSCMessage msg(address.c_str());
  const int32_t receiveAge = lastDaisyRX == 0
      ? -1 : (int32_t)(millis() - lastDaisyRX);
  msg.add((int32_t)systemState.daisy_ok);
  msg.add(receiveAge);
  msg.add((int32_t)daisyCpuLoad);
  msg.add((int32_t)daisyRxPackets);
  msg.add((int32_t)daisyParseErrors);
  msg.add((int32_t)daisyLastError);
  msg.add((int32_t)daisyAudioLevel);
  sendOSC(msg);
}

void sendHapticStatus() {
  String address = String('/') + robotNameStr + "/status/haptics";
  OSCMessage msg(address.c_str());
  msg.add((int32_t)hapticHealthMask);
  for (uint8_t channel = 0; channel < NUM_HAPTICS; ++channel) {
    msg.add((int32_t)hapticFound[channel]);
  }
  sendOSC(msg);
}

void sendOSC(OSCMessage &msg) {
  if (!systemState.wifi_ok || !udpRunning) {
    msg.empty();
    return;
  }

  if (udp.beginPacket(maxIP, maxPort)) {
    msg.send(udp);
    udp.endPacket();
  }
  msg.empty();
}

// ----------------------------
// Daisy UART
// ----------------------------
bool daisyCommandBypassesThrottle(int parameterID) {
  return parameterID == 17
      || parameterID == 19
      || parameterID == 26
      || parameterID == 97;
}

void sendToDaisy(int parameterID, int value) {
  static unsigned long lastSent[100] = {0};

  if (parameterID < 0 || parameterID >= 100) {
    Serial.printf("sendToDaisy: parameter ID %d out of range\n", parameterID);
    return;
  }

  const unsigned long now = millis();
  if (!daisyCommandBypassesThrottle(parameterID)
      && now - lastSent[parameterID] <= 50) {
    return;
  }

  DaisySerial.printf("%d %d\n", parameterID, value);
  lastSent[parameterID] = now;
  lastDaisyTX = now;

#ifdef DEBUG_VERBOSE
  Serial.printf("Sent to Daisy -> ID:%d Val:%d\n", parameterID, value);
#endif
}

void markDaisyReceived() {
  lastDaisyRX = millis();
  ++daisyRxPackets;
  if (!systemState.daisy_ok) {
    systemState.daisy_ok = true;
    Serial.println("Daisy link established");
  }
}

void updateDaisyLink() {
  const unsigned long now = millis();

  if (now - lastDaisyPing >= DAISY_PING_INTERVAL_MS) {
    lastDaisyPing = now;
    DaisySerial.printf("%d 1\n", DAISY_ID_PING);
    lastDaisyTX = now;
  }

  const bool timedOut = lastDaisyRX == 0
      || now - lastDaisyRX > DAISY_LINK_TIMEOUT_MS;
  if (systemState.daisy_ok && timedOut) {
    systemState.daisy_ok = false;
    Serial.println("Daisy link timed out");
  }
}

void readDaisySerial() {
  static String line;
  static bool overflow = false;

  while (DaisySerial.available() > 0) {
    const char character = (char)DaisySerial.read();

    if (character == '\n' || character == '\r') {
      if (!overflow && line.length() > 0) {
        line.trim();
        int id = 0;
        int value = 0;
        char extra = '\0';

        if (sscanf(line.c_str(), " %d %d %c", &id, &value, &extra) == 2) {
          bool recognized = true;
          switch (id) {
            case DAISY_ID_CPU_LOAD:
              daisyCpuLoad = constrain(value, 0, 100);
              lastCpuUpdate = millis();
              break;
            case DAISY_ID_READY:
              Serial.printf("Daisy ready: %d\n", value);
              break;
            case DAISY_ID_HEARTBEAT:
              break;
            case DAISY_ID_ERROR:
              daisyLastError = value;
              Serial.printf("Daisy error/status: %d\n", value);
              break;
            case DAISY_ID_AUDIO:
              daisyAudioLevel = value;
              break;
            default:
              recognized = false;
#ifdef DEBUG_VERBOSE
              Serial.printf("Daisy unknown message: %d %d\n", id, value);
#endif
              break;
          }

          if (recognized) {
            markDaisyReceived();
#ifdef DEBUG_VERBOSE
            Serial.printf("Daisy RX: %d %d\n", id, value);
#endif
          }
        } else {
          ++daisyParseErrors;
          Serial.printf("Daisy malformed line: %s\n", line.c_str());
        }
      }

      line = "";
      overflow = false;
      continue;
    }

    if (overflow) continue;
    if (line.length() < MAX_DAISY_LINE_LENGTH) {
      line += character;
    } else {
      line = "";
      overflow = true;
      ++daisyParseErrors;
    }
  }
}

// ----------------------------
// Haptic playback
// ----------------------------
bool playHapticChannel(uint8_t channel, uint8_t effect) {
  if (channel >= NUM_HAPTICS || !hapticFound[channel]) {
    Serial.printf("Haptic %u is unavailable\n", channel);
    return false;
  }

  effect = constrain(effect, 1, 123);
  if (!selectHapticChannel(channel)) {
    Serial.printf("Haptic %u: mux selection failed\n", channel);
    return false;
  }

  if (channel == PURR_HAPTIC_CHANNEL) suspendPurrForOneShot();

  drv.setMode(DRV2605_MODE_INTTRIG);
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();

#ifdef DEBUG_VERBOSE
  Serial.printf("Haptic %u trigger: effect %u%s\n",
                channel, effect,
                channel == PURR_HAPTIC_CHANNEL ? "; Purr suspended" : "");
#endif
  return true;
}

void playHaptic(int effect) {
  if (!systemState.drv2605_ok) {
    Serial.println("Haptic: no DRV2605 channels available");
    return;
  }

  effect = constrain(effect, 1, 123);
  for (uint8_t channel = 0; channel < NUM_HAPTICS; ++channel) {
    if (hapticFound[channel]) {
      playHapticChannel(channel, (uint8_t)effect);
    }
  }
  disableAllHapticChannels();
}

void playOneHaptic(int channel, int effect) {
  if (channel < 0 || channel >= NUM_HAPTICS) {
    Serial.printf("Haptic: invalid channel %d\n", channel);
    return;
  }
  playHapticChannel((uint8_t)channel,
                    (uint8_t)constrain(effect, 1, 123));
  disableAllHapticChannels();
}

// ----------------------------
// OLED
// ----------------------------
const char* modeLabel() {
  if (modeTransition.active) return "CTR";
  return motionModeShortName(currentMode);
}

int getWiFiBars(int rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

void drawWiFiBars(int x, int y, int bars) {
  for (int index = 0; index < 4; ++index) {
    const int height = (index + 1) * 2;
    const int barX = x + index * 5;
    if (index < bars) {
      display.fillRect(barX, y - height, 3, height, SSD1306_WHITE);
    } else {
      display.drawRect(barX, y - height, 3, height, SSD1306_WHITE);
    }
  }
}

void updateOLED() {
  if (!systemState.display_ok) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  switch (displayMode) {
    case 0: {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(F("SYSTEM READY"));
      drawWiFiBars(100, 8, getWiFiBars(WiFi.RSSI()));

      if (WiFi.status() != WL_CONNECTED) {
        display.setCursor(76, 0);
        display.print("NO WIFI");
      }
      if (millis() - lastOSCActivity < 100) {
        display.fillCircle(90, 4, 2, SSD1306_WHITE);
      }
      if (millis() - lastDaisyTX < 100) {
        display.drawRect(80, 2, 5, 5, SSD1306_WHITE);
      }

      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      display.setCursor(0, 16);
      display.println(F("IP Address:"));
      display.setTextSize(2);
      display.setCursor(0, 26);
      display.println(WiFi.localIP());
      display.setTextSize(1);
      display.setCursor(0, 48);
      display.printf("%s.local", robotName);
      display.setCursor(0, 56);
      display.printf("Port:%u [%s]", localPort, modeLabel());
      display.setCursor(90, 56);
      display.print(systemState.mpr121_ok ? 'T' : '-');
      display.print(systemState.drv2605_ok ? 'H' : '-');
      display.print(systemState.daisy_ok ? 'S' : '-');
      break;
    }

    case 1: {
      display.setTextSize(1);
      display.setCursor(0, 0);

      if (modeTransition.active) {
        display.printf("SERVOS CTR->%s %d%%",
                       motionModeShortName(modeTransition.pendingMode),
                       modeTransitionProgressPercent());
      } else {
        display.printf("SERVOS [%s] A:%d",
                       modeLabel(), activeServoCount);
      }

      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

      for (uint8_t row = 0; row < 4; ++row) {
        const uint8_t leftServo = row * 2;
        const uint8_t rightServo = leftServo + 1;
        const int y = 15 + row * 12;

        display.setCursor(0, y);
        display.printf("S%u:%5.1f", leftServo, servoPos[leftServo]);
        display.setCursor(66, y);
        display.printf("S%u:%5.1f", rightServo, servoPos[rightServo]);
      }
      break;
    }

    case 2:
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(F("TOUCH SENSORS"));
      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      for (uint8_t pad = 0; pad < 12; ++pad) {
        const int x = (pad % 6) * 20;
        const int y = (pad / 6) * 20 + 16;
        const bool touched = currtouched & _BV(pad);
        if (touched) {
          display.fillRect(x, y, 18, 10, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
          display.setCursor(x + 2, y);
          display.printf("%u", pad);
          display.setTextColor(SSD1306_WHITE);
        } else {
          display.setCursor(x, y);
          display.printf(" %u ", pad);
        }
      }
      break;

    case 3:
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(F("NET / IO DEBUG"));
      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      display.setCursor(0, 16);
      display.printf("WiFi: %d dBm", WiFi.RSSI());
      drawWiFiBars(100, 22, getWiFiBars(WiFi.RSSI()));
      display.setCursor(0, 32);
      display.printf("OSC seq: %d", lastSeq);
      display.setCursor(100, 32);
      display.print(millis() - lastOSCActivity < 100 ? "RX*" : "RX");
      display.setCursor(0, 48);
      display.print("Daisy TX:");
      display.setCursor(100, 48);
      display.print(millis() - lastDaisyTX < 100 ? "TX*" : "TX");
      break;

    case 4:
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(F("DAISY CPU"));
      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      if (lastCpuUpdate == 0 || millis() - lastCpuUpdate >= 2000) {
        display.setCursor(0, 20);
        display.print("NO DATA");
      } else {
        display.setTextSize(2);
        display.setCursor(0, 20);
        display.printf("%d%%", daisyCpuLoad);
        const int barWidth = map(daisyCpuLoad, 0, 100, 0, 120);
        display.drawRect(0, 50, 120, 10, SSD1306_WHITE);
        display.fillRect(0, 50, barWidth, 10, SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(90, 20);
        display.print(daisyCpuLoad < 50 ? "OK"
                      : daisyCpuLoad < 80 ? "MID" : "HIGH");
      }
      break;

    case 5:
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(F("SYSTEM HEALTH"));
      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      display.setCursor(0, 16);
      display.printf("WiFi:    %s\n", systemState.wifi_ok ? "OK" : "FAIL");
      display.printf("Touch:   %s\n", systemState.mpr121_ok ? "OK" : "FAIL");
      if (hapticHealthMask == 0x0F) {
        display.println("Haptic:  OK F");
      } else {
        display.printf("Haptic:  MASK %X\n", hapticHealthMask);
      }
      display.printf("Display: %s\n", systemState.display_ok ? "OK" : "FAIL");
      display.printf("Daisy:   %s\n", systemState.daisy_ok ? "OK" : "FAIL");
      if (modeTransition.active) {
        display.printf("CTR->%s %d%%",
                       motionModeShortName(modeTransition.pendingMode),
                       modeTransitionProgressPercent());
      } else {
        display.printf("Mode:%s", modeLabel());
      }
      break;

    default:
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(F("UNKNOWN MODE"));
      break;
  }

  display.display();
}
