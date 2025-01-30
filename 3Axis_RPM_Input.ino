#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// X-Motor is the Leadscrew one
// Y-Motor is the TimerBelt one
// Z-Motor is the TOP-one

// Pin definitions for all three axes
const int X_STEP_PIN = 13; // X axis PUL+ pin
const int X_DIR_PIN = 29;  // X axis DIR+ pin
const int X_EN_PIN = 27;   // X axis ENA+ pin (optional)

const int Y_STEP_PIN = 12; // Y axis PUL+ pin
const int Y_DIR_PIN = 33;  // Y axis DIR+ pin
const int Y_EN_PIN = 35;   // Y axis ENA+ pin (optional)

const int Z_STEP_PIN = 11; // Z axis PUL+ pin
const int Z_DIR_PIN = 24;  // Z axis DIR+ pin
const int Z_EN_PIN = 22;   // Z axis ENA+ pin (optional)

// Voltage feedback pin for Z-axis
const int voltagePin = A0; // Analog pin for voltage sensing

// Motor parameters
const int motorStepsPerRevolution = 200; // Steps per revolution (standard motor)

// Microstepping settings for each axis (Current using DIP => 1.5 - 1.7 amps)
const int xMicrosteps = 16; // Microsteps setting for X-axis TB6600 (DIP = 001110)
const int yMicrosteps = 1;  // Microsteps setting for Y-axis TB6600 (DIP = 110110)(More Torque Required)
const int zMicrosteps = 32; // Microsteps setting for Z-axis TB6600 (DIP = 000110)

// Effective steps per revolution for each axis
const int xStepsPerRevolution = motorStepsPerRevolution * xMicrosteps;
const int yStepsPerRevolution = motorStepsPerRevolution * yMicrosteps;
const int zStepsPerRevolution = motorStepsPerRevolution * zMicrosteps;

// Variables to hold user input for speed and direction
int xRpm = 0, yRpm = 0, zRpm = 0;                          // Speed in RPM for each axis
int xStepDelay = 0, yStepDelay = 0, zStepDelay = 0;        // Delays in microseconds between steps
char xDirection = 'F', yDirection = 'F', zDirection = 'F'; // Directions for each axis
bool xRunning = false, yRunning = false, zRunning = false; // Motor running states

// Flags for modes
bool closedLoopMode = false;

// LCD parameters
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

// Coordinates
int xCoord = 0;
int yCoord = 0;
int zCoord = 0;

// Voltage variables for Z-axis feedback
const float arduinoVCC = 5.01; // Arduino voltage
unsigned long ValueR1 = 98700; // Resistor R1 value
unsigned long ValueR2 = 11740; // Resistor R2 value
float voltage = 0.0;           // Sensed voltage
const int inputResolution = 1023;

void setup()
{
  // Set all pins as outputs
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_EN_PIN, OUTPUT);

  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_EN_PIN, OUTPUT);

  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_EN_PIN, OUTPUT);

  pinMode(voltagePin, INPUT); // Voltage sensing pin

  // Enable all motors (optional)
  digitalWrite(X_EN_PIN, LOW); // LOW to enable
  digitalWrite(Y_EN_PIN, LOW); // LOW to enable
  digitalWrite(Z_EN_PIN, LOW); // LOW to enable

  // Start serial communication
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("X:0 Y:0 Z:0");
  lcd.setCursor(0, 1);
  lcd.print("V:0.00 Mode:Open");

  // Print instructions
  printHelp();
}

void loop()
{
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // Run motors for each axis if enabled
  if (xRunning)
  {
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(xStepDelay / 2);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(xStepDelay / 2);
  }
  if (yRunning)
  {
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(yStepDelay / 2);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(yStepDelay / 2);
  }
  if (zRunning)
  {
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(zStepDelay / 2);
    digitalWrite(Z_STEP_PIN, LOW);
    delayMicroseconds(zStepDelay / 2);
  }

  // If in closed-loop mode, perform Z-axis control
  if (closedLoopMode)
  {
    closedLoopControlZ();
  }
}

void printHelp()
{
  Serial.println("\n=== 3-Axis Control Instructions ===");
  Serial.println("Commands (Everything is Case Insensitive):");
  Serial.println("XxxF : Set RPM for X-axis to xx and direction Forward");
  Serial.println("YxxF : Set RPM for Y-axis to xx and direction Forward");
  Serial.println("ZxxF : Set RPM for Z-axis to xx and direction Forward");
  Serial.println("XxxB : Set RPM for X-axis to xx and direction Backward");
  Serial.println("YxxB : Set RPM for Y-axis to xx and direction Backward");
  Serial.println("ZxxB : Set RPM for Z-axis to xx and direction Backward");
  Serial.println("S    : Stop all movement");
  Serial.println("H    : Show this help message");
  Serial.println("===============================");
}

void processCommand(String command)
{
  command.trim(); // Remove any whitespace

  // Help command
  if (command.equalsIgnoreCase("H"))
  {
    printHelp();
    return;
  }

  // Stop command
  if (command.equalsIgnoreCase("S"))
  {
    xRunning = yRunning = zRunning = false;
    closedLoopMode = false;
    Serial.println("Stopped all movement and exited closed-loop mode.");
    return;
  }

  // RPM and direction commands for each axis
  if (command.startsWith("X") || command.startsWith("Y") || command.startsWith("Z"))
  {
    char axis = command.charAt(0);
    int rpm = command.substring(1, command.length() - 1).toInt();
    rpm = constrain(rpm, 1, 1000); // Constrain RPM to a valid range

    char dir = command.charAt(command.length() - 1);
    if (dir == 'F' || dir == 'B')
    {
      int stepDelay;
      bool running = true;

      if (axis == 'X')
      {
        stepDelay = 60000000 / (rpm * xStepsPerRevolution); // Calculate delay per step
        xRpm = rpm;
        xStepDelay = stepDelay;
        xDirection = dir;
        digitalWrite(X_DIR_PIN, (dir == 'F') ? HIGH : LOW);
        xRunning = running;
      }
      else if (axis == 'Y')
      {
        stepDelay = 60000000 / (rpm * yStepsPerRevolution); // Calculate delay per step
        yRpm = rpm;
        yStepDelay = stepDelay;
        yDirection = dir;
        digitalWrite(Y_DIR_PIN, (dir == 'F') ? HIGH : LOW);
        yRunning = running;
      }
      else if (axis == 'Z')
      {
        stepDelay = 60000000 / (rpm * zStepsPerRevolution); // Calculate delay per step
        zRpm = rpm;
        zStepDelay = stepDelay;
        zDirection = dir;
        digitalWrite(Z_DIR_PIN, (dir == 'F') ? HIGH : LOW);
        zRunning = running;
      }

      Serial.print(axis);
      Serial.print("-axis speed set to: ");
      Serial.print(rpm);
      Serial.print(" RPM, direction: ");
      Serial.println((dir == 'F') ? "Forward" : "Backward");
    }
    else
    {
      Serial.println("Invalid direction. Use 'F' for forward or 'B' for backward.");
    }
    return;
  }

  Serial.println("Invalid command. Type 'H' for help.");
}

void readVoltage()
{
  int A0Value = analogRead(voltagePin);
  float voltage_sensed = A0Value * (arduinoVCC / (float)inputResolution);
  voltage = 10 * voltage_sensed * (1 + ((float)ValueR2 / (float)ValueR1));
}

void closedLoopControlZ()
{
  readVoltage();

  if (voltage >= 20.0)
  {
    // Move Z-axis downward until voltage drops
    digitalWrite(Z_DIR_PIN, LOW); // Downward direction
    while (voltage >= 20.0 && closedLoopMode)
    {
      digitalWrite(Z_STEP_PIN, HIGH);
      delayMicroseconds(zStepDelay);
      digitalWrite(Z_STEP_PIN, LOW);
      delayMicroseconds(zStepDelay);
      readVoltage();
      zCoord--;
      updateLCD();
    }
  }
  else
  {
    // Move Z-axis upward until voltage rises
    digitalWrite(Z_DIR_PIN, HIGH); // Upward direction
    while (voltage < 20.0 && closedLoopMode)
    {
      digitalWrite(Z_STEP_PIN, HIGH);
      delayMicroseconds(zStepDelay);
      digitalWrite(Z_STEP_PIN, LOW);
      delayMicroseconds(zStepDelay);
      readVoltage();
      zCoord++;
      updateLCD();
    }
  }

  Serial.println("Closed-loop control complete for current condition.");
}

void updateLCD()
{
  // Clear the previous values
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.print(xCoord);
  lcd.print(" Y:");
  lcd.print(yCoord);
  lcd.print(" Z:");
  lcd.print(zCoord);
  lcd.print("   "); // Overwrite any leftover characters

  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(voltage, 2); // Print voltage with 2 decimal places
  lcd.print(" Mode:");
  lcd.print(closedLoopMode ? "ClsdZ" : "Open");
}
