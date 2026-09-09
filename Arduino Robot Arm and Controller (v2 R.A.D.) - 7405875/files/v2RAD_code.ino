#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>

// ==================================================
// PCA9685 configuration
// ==================================================

constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint16_t SERVO_FREQUENCY = 50;

Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);
Preferences preferences;

// ==================================================
// Nano ESP32 potentiometer pins
// ==================================================

constexpr int POT_BASE     = A0;
constexpr int POT_SHOULDER = A1;
constexpr int POT_ELBOW    = A2;
constexpr int POT_WRIST    = A3;

// Nano ESP32 I2C:
// A4 = SDA
// A5 = SCL

// Button connected between D2 and GND
constexpr int BUTTON_PIN = D2;

// ==================================================
// PCA9685 servo channels
// ==================================================

constexpr uint8_t SERVO_BASE     = 4;
constexpr uint8_t SERVO_SHOULDER = 3;
constexpr uint8_t SERVO_ELBOW    = 2;
constexpr uint8_t SERVO_WRIST    = 1;
constexpr uint8_t SERVO_GRIPPER  = 0;

// ==================================================
// Potentiometer calibration
// ==================================================

constexpr int POT_MIN_READING = 500;
constexpr int POT_MAX_READING = 3300;

// ==================================================
// Arm servo pulse calibration
// ==================================================

constexpr uint16_t ARM_MIN_US = 500;
constexpr uint16_t ARM_MAX_US = 2500;

// Default starting position on the first-ever startup
constexpr uint16_t DEFAULT_START_PULSE_US = 1500;

// ==================================================
// Movement smoothing settings
// ==================================================

// Arm servo commands update once per 50 Hz servo cycle.
constexpr unsigned long ARM_UPDATE_INTERVAL_MS = 20;

// Lower value = smoother but slower response.
// Higher value = faster but less smoothing.
// Useful range: 0.10 to 0.40.
constexpr float POT_SMOOTHING_ALPHA = 0.18f;

// Ignore pulse changes smaller than this.
// Increase slightly if the servos still jitter.
constexpr int SERVO_DEADBAND_US = 4;

unsigned long previousArmUpdateTime = 0;

// Filtered potentiometer values
float filteredBasePot = 0.0f;
float filteredShoulderPot = 0.0f;
float filteredElbowPot = 0.0f;
float filteredWristPot = 0.0f;

bool potFiltersInitialized = false;

// ==================================================
// Startup soft-movement settings
// ==================================================

constexpr unsigned long STARTUP_RAMP_DURATION_MS = 3000;
constexpr unsigned long STARTUP_RAMP_STEP_MS = 20;

// Time to position the controller before startup motion
constexpr unsigned long CONTROLLER_POSITION_DELAY_MS = 5000;

// ==================================================
// Position-saving settings
// ==================================================

constexpr unsigned long POSITION_SAVE_INTERVAL_MS = 10000;
constexpr int POSITION_SAVE_THRESHOLD_US = 5;

unsigned long previousPositionSaveTime = 0;

// Current arm output pulse values
uint16_t currentBasePulse = DEFAULT_START_PULSE_US;
uint16_t currentShoulderPulse = DEFAULT_START_PULSE_US;
uint16_t currentElbowPulse = DEFAULT_START_PULSE_US;
uint16_t currentWristPulse = DEFAULT_START_PULSE_US;

// Last values written to memory
uint16_t lastSavedBasePulse = DEFAULT_START_PULSE_US;
uint16_t lastSavedShoulderPulse = DEFAULT_START_PULSE_US;
uint16_t lastSavedElbowPulse = DEFAULT_START_PULSE_US;
uint16_t lastSavedWristPulse = DEFAULT_START_PULSE_US;

int lastSavedGripperAngle = 180;
bool lastSavedGripperClosing = true;

// ==================================================
// Serial print settings
// ==================================================

constexpr unsigned long POT_PRINT_INTERVAL_MS = 200;
unsigned long previousPotPrintTime = 0;

// ==================================================
// Gripper settings
// ==================================================

constexpr uint16_t GRIPPER_MIN_US = 900;
constexpr uint16_t GRIPPER_MAX_US = 2100;

constexpr int GRIPPER_CLOSED_ANGLE = 20;
constexpr int GRIPPER_OPEN_ANGLE   = 180;

constexpr int GRIPPER_STEP_DEGREES = 5;
constexpr unsigned long GRIPPER_STEP_INTERVAL_MS = 40;

int gripperAngle = GRIPPER_OPEN_ANGLE;

// First button hold closes the gripper
bool gripperClosing = true;

unsigned long previousGripperMoveTime = 0;

// ==================================================
// Button debounce settings
// ==================================================

constexpr unsigned long BUTTON_DEBOUNCE_MS = 25;

bool previousRawButtonState = false;
bool stableButtonState = false;

unsigned long buttonChangeTime = 0;

// ==================================================
// Axis directions
// ==================================================

constexpr bool REVERSE_BASE     = true;
constexpr bool REVERSE_SHOULDER = true;
constexpr bool REVERSE_ELBOW    = true;
constexpr bool REVERSE_WRIST    = false;

// ==================================================
// Read and average one potentiometer
// ==================================================

int readPotentiometer(int pin)
{
  constexpr int SAMPLE_COUNT = 8;
  uint32_t total = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++)
  {
    total += analogRead(pin);
  }

  return total / SAMPLE_COUNT;
}

// ==================================================
// Initialize smoothing filters
// ==================================================

void initializePotFilters(
  int baseValue,
  int shoulderValue,
  int elbowValue,
  int wristValue
)
{
  filteredBasePot = baseValue;
  filteredShoulderPot = shoulderValue;
  filteredElbowPot = elbowValue;
  filteredWristPot = wristValue;

  potFiltersInitialized = true;
}

// ==================================================
// Apply exponential smoothing to a pot reading
// ==================================================

float smoothPotReading(
  float previousFilteredValue,
  int newRawValue
)
{
  return previousFilteredValue +
         POT_SMOOTHING_ALPHA *
         (newRawValue - previousFilteredValue);
}

// ==================================================
// Convert potentiometer reading to servo pulse
// ==================================================

uint16_t potReadingToPulse(
  int reading,
  bool reverseDirection
)
{
  reading = constrain(
    reading,
    POT_MIN_READING,
    POT_MAX_READING
  );

  if (reverseDirection)
  {
    reading =
      POT_MIN_READING +
      POT_MAX_READING -
      reading;
  }

  return static_cast<uint16_t>(
    map(
      reading,
      POT_MIN_READING,
      POT_MAX_READING,
      ARM_MIN_US,
      ARM_MAX_US
    )
  );
}

// ==================================================
// Command all four arm servos
// ==================================================

void commandArmServos(
  uint16_t basePulse,
  uint16_t shoulderPulse,
  uint16_t elbowPulse,
  uint16_t wristPulse
)
{
  pwm.writeMicroseconds(SERVO_BASE, basePulse);
  pwm.writeMicroseconds(SERVO_SHOULDER, shoulderPulse);
  pwm.writeMicroseconds(SERVO_ELBOW, elbowPulse);
  pwm.writeMicroseconds(SERVO_WRIST, wristPulse);

  currentBasePulse = basePulse;
  currentShoulderPulse = shoulderPulse;
  currentElbowPulse = elbowPulse;
  currentWristPulse = wristPulse;
}

// ==================================================
// Update one servo with filtered pot input
// ==================================================

void updateSmoothedServo(
  int rawPotReading,
  float &filteredPotReading,
  uint8_t servoChannel,
  bool reverseDirection,
  uint16_t &currentPulse
)
{
  filteredPotReading = smoothPotReading(
    filteredPotReading,
    rawPotReading
  );

  uint16_t targetPulse = potReadingToPulse(
    static_cast<int>(filteredPotReading),
    reverseDirection
  );

  int pulseDifference =
    static_cast<int>(targetPulse) -
    static_cast<int>(currentPulse);

  // Ignore tiny commands caused by ADC noise
  if (abs(pulseDifference) < SERVO_DEADBAND_US)
  {
    return;
  }

  currentPulse = targetPulse;

  pwm.writeMicroseconds(
    servoChannel,
    currentPulse
  );
}

// ==================================================
// Update all arm servos with smoothing
// ==================================================

void updateSmoothedArm(
  int basePotValue,
  int shoulderPotValue,
  int elbowPotValue,
  int wristPotValue
)
{
  if (!potFiltersInitialized)
  {
    initializePotFilters(
      basePotValue,
      shoulderPotValue,
      elbowPotValue,
      wristPotValue
    );
  }

  if (
    millis() - previousArmUpdateTime <
    ARM_UPDATE_INTERVAL_MS
  )
  {
    return;
  }

  previousArmUpdateTime = millis();

  updateSmoothedServo(
    basePotValue,
    filteredBasePot,
    SERVO_BASE,
    REVERSE_BASE,
    currentBasePulse
  );

  updateSmoothedServo(
    shoulderPotValue,
    filteredShoulderPot,
    SERVO_SHOULDER,
    REVERSE_SHOULDER,
    currentShoulderPulse
  );

  updateSmoothedServo(
    elbowPotValue,
    filteredElbowPot,
    SERVO_ELBOW,
    REVERSE_ELBOW,
    currentElbowPulse
  );

  updateSmoothedServo(
    wristPotValue,
    filteredWristPot,
    SERVO_WRIST,
    REVERSE_WRIST,
    currentWristPulse
  );
}

// ==================================================
// Print raw potentiometer readings
// ==================================================

void printPotValues(
  int baseValue,
  int shoulderValue,
  int elbowValue,
  int wristValue
)
{
  if (
    millis() - previousPotPrintTime <
    POT_PRINT_INTERVAL_MS
  )
  {
    return;
  }

  previousPotPrintTime = millis();

  Serial.print("Base: ");
  Serial.print(baseValue);

  Serial.print(" | Shoulder: ");
  Serial.print(shoulderValue);

  Serial.print(" | Elbow: ");
  Serial.print(elbowValue);

  Serial.print(" | Wrist: ");
  Serial.println(wristValue);
}

// ==================================================
// Gripper functions
// ==================================================

uint16_t gripperAngleToPulse(int angle)
{
  angle = constrain(angle, 0, 180);

  return static_cast<uint16_t>(
    map(
      angle,
      0,
      180,
      GRIPPER_MIN_US,
      GRIPPER_MAX_US
    )
  );
}

void setGripperAngle(int angle)
{
  gripperAngle = constrain(
    angle,
    GRIPPER_CLOSED_ANGLE,
    GRIPPER_OPEN_ANGLE
  );

  pwm.writeMicroseconds(
    SERVO_GRIPPER,
    gripperAngleToPulse(gripperAngle)
  );
}

// ==================================================
// Load last positions from ESP32 memory
// ==================================================

void loadSavedPositions()
{
  preferences.begin("robot-arm", false);

  currentBasePulse = constrain(
    preferences.getUShort(
      "base",
      DEFAULT_START_PULSE_US
    ),
    ARM_MIN_US,
    ARM_MAX_US
  );

  currentShoulderPulse = constrain(
    preferences.getUShort(
      "shoulder",
      DEFAULT_START_PULSE_US
    ),
    ARM_MIN_US,
    ARM_MAX_US
  );

  currentElbowPulse = constrain(
    preferences.getUShort(
      "elbow",
      DEFAULT_START_PULSE_US
    ),
    ARM_MIN_US,
    ARM_MAX_US
  );

  currentWristPulse = constrain(
    preferences.getUShort(
      "wrist",
      DEFAULT_START_PULSE_US
    ),
    ARM_MIN_US,
    ARM_MAX_US
  );

  gripperAngle = constrain(
    preferences.getUShort(
      "gripper",
      GRIPPER_OPEN_ANGLE
    ),
    GRIPPER_CLOSED_ANGLE,
    GRIPPER_OPEN_ANGLE
  );

  gripperClosing = preferences.getBool(
    "closing",
    true
  );

  lastSavedBasePulse = currentBasePulse;
  lastSavedShoulderPulse = currentShoulderPulse;
  lastSavedElbowPulse = currentElbowPulse;
  lastSavedWristPulse = currentWristPulse;

  lastSavedGripperAngle = gripperAngle;
  lastSavedGripperClosing = gripperClosing;
}

// ==================================================
// Save last commanded positions
// ==================================================

void savePositionsIfNeeded()
{
  if (
    millis() - previousPositionSaveTime <
    POSITION_SAVE_INTERVAL_MS
  )
  {
    return;
  }

  previousPositionSaveTime = millis();

  bool armPositionChanged =
    abs(
      static_cast<int>(currentBasePulse) -
      static_cast<int>(lastSavedBasePulse)
    ) >= POSITION_SAVE_THRESHOLD_US ||
    abs(
      static_cast<int>(currentShoulderPulse) -
      static_cast<int>(lastSavedShoulderPulse)
    ) >= POSITION_SAVE_THRESHOLD_US ||
    abs(
      static_cast<int>(currentElbowPulse) -
      static_cast<int>(lastSavedElbowPulse)
    ) >= POSITION_SAVE_THRESHOLD_US ||
    abs(
      static_cast<int>(currentWristPulse) -
      static_cast<int>(lastSavedWristPulse)
    ) >= POSITION_SAVE_THRESHOLD_US;

  bool gripperPositionChanged =
    gripperAngle != lastSavedGripperAngle ||
    gripperClosing != lastSavedGripperClosing;

  if (!armPositionChanged && !gripperPositionChanged)
  {
    return;
  }

  preferences.putUShort("base", currentBasePulse);
  preferences.putUShort("shoulder", currentShoulderPulse);
  preferences.putUShort("elbow", currentElbowPulse);
  preferences.putUShort("wrist", currentWristPulse);

  preferences.putUShort(
    "gripper",
    gripperAngle
  );

  preferences.putBool(
    "closing",
    gripperClosing
  );

  lastSavedBasePulse = currentBasePulse;
  lastSavedShoulderPulse = currentShoulderPulse;
  lastSavedElbowPulse = currentElbowPulse;
  lastSavedWristPulse = currentWristPulse;

  lastSavedGripperAngle = gripperAngle;
  lastSavedGripperClosing = gripperClosing;

  Serial.println("Servo positions saved");
}

// ==================================================
// Slowly move from saved positions to pot positions
// ==================================================

void performStartupRamp()
{
  int basePotValue = readPotentiometer(POT_BASE);
  int shoulderPotValue = readPotentiometer(POT_SHOULDER);
  int elbowPotValue = readPotentiometer(POT_ELBOW);
  int wristPotValue = readPotentiometer(POT_WRIST);

  uint16_t targetBasePulse = potReadingToPulse(
    basePotValue,
    REVERSE_BASE
  );

  uint16_t targetShoulderPulse = potReadingToPulse(
    shoulderPotValue,
    REVERSE_SHOULDER
  );

  uint16_t targetElbowPulse = potReadingToPulse(
    elbowPotValue,
    REVERSE_ELBOW
  );

  uint16_t targetWristPulse = potReadingToPulse(
    wristPotValue,
    REVERSE_WRIST
  );

  uint16_t startingBasePulse = currentBasePulse;
  uint16_t startingShoulderPulse = currentShoulderPulse;
  uint16_t startingElbowPulse = currentElbowPulse;
  uint16_t startingWristPulse = currentWristPulse;

  Serial.println("Beginning startup soft movement");

  unsigned long rampStartTime = millis();

  while (true)
  {
    unsigned long elapsedTime =
      millis() - rampStartTime;

    float progress =
      static_cast<float>(elapsedTime) /
      static_cast<float>(STARTUP_RAMP_DURATION_MS);

    if (progress > 1.0f)
    {
      progress = 1.0f;
    }

    uint16_t newBasePulse =
      startingBasePulse +
      static_cast<int32_t>(
        targetBasePulse - startingBasePulse
      ) * progress;

    uint16_t newShoulderPulse =
      startingShoulderPulse +
      static_cast<int32_t>(
        targetShoulderPulse - startingShoulderPulse
      ) * progress;

    uint16_t newElbowPulse =
      startingElbowPulse +
      static_cast<int32_t>(
        targetElbowPulse - startingElbowPulse
      ) * progress;

    uint16_t newWristPulse =
      startingWristPulse +
      static_cast<int32_t>(
        targetWristPulse - startingWristPulse
      ) * progress;

    commandArmServos(
      newBasePulse,
      newShoulderPulse,
      newElbowPulse,
      newWristPulse
    );

    if (progress >= 1.0f)
    {
      break;
    }

    delay(STARTUP_RAMP_STEP_MS);
  }

  Serial.println("Startup soft movement complete");
}

// ==================================================
// Read and debounce button
// ==================================================

void updateButton()
{
  bool rawButtonState =
    digitalRead(BUTTON_PIN) == LOW;

  if (rawButtonState != previousRawButtonState)
  {
    previousRawButtonState = rawButtonState;
    buttonChangeTime = millis();
  }

  if (
    millis() - buttonChangeTime >= BUTTON_DEBOUNCE_MS &&
    rawButtonState != stableButtonState
  )
  {
    bool previousStableState = stableButtonState;
    stableButtonState = rawButtonState;

    // Change direction when button is released
    if (previousStableState && !stableButtonState)
    {
      gripperClosing = !gripperClosing;

      if (gripperClosing)
      {
        Serial.println("Next gripper movement: close");
      }
      else
      {
        Serial.println("Next gripper movement: open");
      }
    }
  }
}

// ==================================================
// Move gripper while button is held
// ==================================================

void updateGripper()
{
  if (!stableButtonState)
  {
    return;
  }

  if (
    millis() - previousGripperMoveTime <
    GRIPPER_STEP_INTERVAL_MS
  )
  {
    return;
  }

  previousGripperMoveTime = millis();

  if (gripperClosing)
  {
    if (gripperAngle > GRIPPER_CLOSED_ANGLE)
    {
      setGripperAngle(
        max(
          GRIPPER_CLOSED_ANGLE,
          gripperAngle - GRIPPER_STEP_DEGREES
        )
      );
    }
  }
  else
  {
    if (gripperAngle < GRIPPER_OPEN_ANGLE)
    {
      setGripperAngle(
        min(
          GRIPPER_OPEN_ANGLE,
          gripperAngle + GRIPPER_STEP_DEGREES
        )
      );
    }
  }
}

// ==================================================
// Setup
// ==================================================

void setup()
{
  Serial.begin(115200);

  analogReadResolution(12);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQUENCY);

  delay(10);

  // Load the last positions commanded before shutdown
  loadSavedPositions();

  // Begin at the saved positions
  commandArmServos(
    currentBasePulse,
    currentShoulderPulse,
    currentElbowPulse,
    currentWristPulse
  );

  setGripperAngle(gripperAngle);

  Serial.println("Robot arm powered");
  Serial.println("Position controller for startup");

  delay(CONTROLLER_POSITION_DELAY_MS);

  // Slowly move from saved positions to current pots
  performStartupRamp();

  // Initialize filters to the current controller position
  initializePotFilters(
    readPotentiometer(POT_BASE),
    readPotentiometer(POT_SHOULDER),
    readPotentiometer(POT_ELBOW),
    readPotentiometer(POT_WRIST)
  );

  previousArmUpdateTime = millis();
  previousPositionSaveTime = millis();
  previousGripperMoveTime = millis();

  Serial.println("Robot arm ready");
}

// ==================================================
// Main loop
// ==================================================

void loop()
{
  int basePotValue = readPotentiometer(POT_BASE);
  int shoulderPotValue = readPotentiometer(POT_SHOULDER);
  int elbowPotValue = readPotentiometer(POT_ELBOW);
  int wristPotValue = readPotentiometer(POT_WRIST);

  // Track the pots with smoothing
  updateSmoothedArm(
    basePotValue,
    shoulderPotValue,
    elbowPotValue,
    wristPotValue
  );

  printPotValues(
    basePotValue,
    shoulderPotValue,
    elbowPotValue,
    wristPotValue
  );

  updateButton();
  updateGripper();

  savePositionsIfNeeded();

  delay(1);
}