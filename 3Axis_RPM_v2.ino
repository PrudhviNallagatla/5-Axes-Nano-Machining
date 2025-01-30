#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================ Hardware Configuration ================
// TB6600 DIP Settings (Match physical switches)
const int X_MICROSTEPS = 16;   // 1/16 microstepping (DIP: 001110)
const int Y_MICROSTEPS = 1;    // 1 microstepping  (DIP: 110110)
const int Z_MICROSTEPS = 32;   // 1/32 microstepping (DIP: 000110)

// Pin Definitions (Triple-check connections!)
const int X_STEP = 13, X_DIR = 29, X_EN = 27;
const int Y_STEP = 12, Y_DIR = 33, Y_EN = 35;
const int Z_STEP = 11, Z_DIR = 24, Z_EN = 22;
const int VOLTAGE_PIN = A0;

// Voltage Divider (Calibrated for your resistors)
const float ARDUINO_VCC = 5.01;  // Measure actual 5V pin voltage
const long R1 = 98700, R2 = 11740;

// ================ Global Variables ================
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Confirm I2C address

// Motor Parameters
const int STEPS_PER_REV = 200;       // NEMA 17 standard
int xTargetRPM = 0, yTargetRPM = 0, zTargetRPM = 0;
unsigned long xStepDelay = 0, yStepDelay = 0, zStepDelay = 0;
volatile long xPos = 0, yPos = 0, zPos = 0;

// Timing
unsigned long xLastStep = 0, yLastStep = 0, zLastStep = 0;
unsigned long voltageLastRead = 0;

// Modes
bool closedLoopZ = false;
float currentVoltage = 0.0;

// ================ Setup ================
void setup() {
  // Initialize motor control pins
  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT); pinMode(X_EN, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT); pinMode(Y_EN, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT); pinMode(Z_EN, OUTPUT);

  // Critical Enable Pin Configuration (LOW = Enable for most TB6600)
  digitalWrite(X_EN, LOW);  // Enable X motor
  digitalWrite(Y_EN, LOW);  // Enable Y motor
  digitalWrite(Z_EN, LOW);  // Enable Z motor

  // LCD Initialization
  lcd.init();
  lcd.backlight();
  updateLCD();

  // Serial Communication
  Serial.begin(115200);
  Serial.println("CNC Controller Ready");
  printHelp();
}

// ================ Main Loop ================
void loop() {
  handleSerial();
  runMotors();
  if(closedLoopZ && millis() - voltageLastRead > 100) {
    readVoltage();
    controlZClosedLoop();
    voltageLastRead = millis();
  }
}

// ================ Motor Control Core ================
void runMotors() {
  unsigned long now = micros();

  // X Axis Movement
  if(xTargetRPM > 0 && (now - xLastStep) >= xStepDelay) {
    digitalWrite(X_STEP, HIGH);      // Explicit pulse generation
    delayMicroseconds(5);            // 5μs pulse width (TB6600 minimum: 2.5μs)
    digitalWrite(X_STEP, LOW);
    xLastStep = now;
    xPos += (digitalRead(X_DIR) == HIGH) ? 1 : -1;
  }

  // Y Axis Movement
  if(yTargetRPM > 0 && (now - yLastStep) >= yStepDelay) {
    digitalWrite(Y_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(Y_STEP, LOW);
    yLastStep = now;
    yPos += (digitalRead(Y_DIR) == HIGH) ? 1 : -1;
  }

  // Z Axis Movement
  if(zTargetRPM > 0 && (now - zLastStep) >= zStepDelay) {
    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(Z_STEP, LOW);
    zLastStep = now;
    zPos += (digitalRead(Z_DIR) == HIGH) ? 1 : -1;
  }
}

// ================ Serial Command Handling ================
void handleSerial() {
  if(Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if(cmd == "S") {  // Emergency stop
      xTargetRPM = yTargetRPM = zTargetRPM = 0;
      closedLoopZ = false;
      Serial.println("!EMERGENCY STOP!");
    }
    else if(cmd == "H") printHelp();
    else if(cmd == "C") {
      closedLoopZ = !closedLoopZ;
      Serial.print("Z Closed-Loop: ");
      Serial.println(closedLoopZ ? "ENABLED" : "DISABLED");
    }
    else if(cmd.length() >= 3) processMotorCommand(cmd);
    else Serial.println("?INVALID COMMAND");

    updateLCD();
  }
}

// ================ Closed-Loop Voltage Control ================
void controlZClosedLoop() {
  if(currentVoltage >= 20.0) {
    digitalWrite(Z_DIR, HIGH);  // Move UP to reduce voltage
    zTargetRPM = 10;
    Serial.println("Z Moving UP (Voltage High)");
  } else {
    digitalWrite(Z_DIR, LOW);   // Move DOWN to increase voltage
    zTargetRPM = 10;
    Serial.println("Z Moving DOWN (Voltage Low)");
  }

  // Deadzone to prevent hunting
  if(abs(currentVoltage - 20.0) < 0.5) {
    zTargetRPM = 0;
    Serial.println("Z Stable");
  }
}

// ================ Command Processing ================
void processMotorCommand(String cmd) {
  char axis = cmd[0];
  int rpm = cmd.substring(1, cmd.length()-1).toInt();
  char dir = cmd.charAt(cmd.length()-1);

  if(rpm < 1 || rpm > 1000) {
    Serial.println("?RPM RANGE: 1-1000");
    return;
  }

  switch(axis) {
    case 'X':
      digitalWrite(X_DIR, (dir == 'F') ? HIGH : LOW);
      xTargetRPM = rpm;
      xStepDelay = 60000000UL / (rpm * STEPS_PER_REV * X_MICROSTEPS);
      Serial.print("X: ");
      break;
    case 'Y':
      digitalWrite(Y_DIR, (dir == 'F') ? HIGH : LOW);
      yTargetRPM = rpm;
      yStepDelay = 60000000UL / (rpm * STEPS_PER_REV * Y_MICROSTEPS);
      Serial.print("Y: ");
      break;
    case 'Z':
      digitalWrite(Z_DIR, (dir == 'F') ? HIGH : LOW);
      zTargetRPM = rpm;
      zStepDelay = 60000000UL / (rpm * STEPS_PER_REV * Z_MICROSTEPS);
      Serial.print("Z: ");
      break;
    default:
      Serial.println("?INVALID AXIS");
      return;
  }
  Serial.print(rpm); Serial.println(" RPM");
}

// ================ Voltage Measurement ================
void readVoltage() {
  const int samples = 10;
  float avg = 0;
  for(int i=0; i<samples; i++) {
    avg += analogRead(VOLTAGE_PIN);
    delay(2);
  }
  avg /= samples;
  currentVoltage = (avg * ARDUINO_VCC / 1023.0) * (1 + (float)R2/R1) * 10;
  Serial.print("Voltage: "); Serial.println(currentVoltage, 1);
}

// ================ User Interface ================
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.print(xPos);
  lcd.print(" Y:");
  lcd.print(yPos);

  lcd.setCursor(0, 1);
  lcd.print("Z:");
  lcd.print(zPos);
  lcd.print(" V:");
  lcd.print(currentVoltage, 1);
  lcd.print(closedLoopZ ? " CL" : " OL");
}

void printHelp() {
  Serial.println("\n=== CNC CONTROLS ===");
  Serial.println("X10F  - X Axis 10 RPM Forward");
  Serial.println("Y5B   - Y Axis 5 RPM Backward");
  Serial.println("Z20F  - Z Axis 20 RPM Forward");
  Serial.println("C     - Toggle Z closed-loop");
  Serial.println("S     - Emergency stop");
  Serial.println("H     - Show help");
}
