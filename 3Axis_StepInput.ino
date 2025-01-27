#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions for all three axes
const int X_STEP_PIN = 13;  // X axis PUL+ pin
const int X_DIR_PIN = 29;   // X axis DIR+ pin
const int X_EN_PIN = 27;    // X axis ENA+ pin (optional)

const int Y_STEP_PIN = 12;  // Y axis PUL+ pin
const int Y_DIR_PIN = 33;   // Y axis DIR+ pin
const int Y_EN_PIN = 35;    // Y axis ENA+ pin (optional)

const int Z_STEP_PIN = 11;  // Z axis PUL+ pin
const int Z_DIR_PIN = 24;   // Z axis DIR+ pin
const int Z_EN_PIN = 22;    // Z axis ENA+ pin (optional)

// Voltage feedback pin for Z-axis
const int voltagePin = A0;  // Analog pin for voltage sensing

// Motor parameters
const int STEP_DELAY = 1000;  // Delay between steps (microseconds)
bool isMoving = false;       // Flag to track if any motor is moving

// Voltage variables for Z-axis feedback
const float arduinoVCC = 5.01; // Arduino voltage
unsigned long ValueR1 = 98700; // Resistor R1 value
unsigned long ValueR2 = 11740;  // Resistor R2 value
float voltage = 0.0;           // Sensed voltage
const int inputResolution = 1023;

// // Define the number of steps per revolution for your motor
// const int motorStepsPerRevolution = 200; // Motor steps per revolution (assuming standard motor)
// const int microsteps = 32; // Microsteps setting for TB6600
// const int stepsPerRevolution = 6400; // Total steps per revolution with microstepping
// // Variables to hold user input for speed and direction
// int speed = 0; // Speed in microseconds between steps
// char direction = 'F'; // 'F' for forward, 'B' for backward
// bool running = false; // Motor running state 

// Flags for modes
bool closedLoopMode = false;

// LCD parameters
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address to 0x27 for a 16 chars and 2 line display

// Coordinates
int xCoord = 0;
int yCoord = 0;
int zCoord = 0;

// enum ControlMode {
//   OPEN_LOOP,
//   CLOSED_LOOP_Z
// };
// ControlMode currentMode = OPEN_LOOP;

void setup() {
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

  pinMode(voltagePin, INPUT);  // Voltage sensing pin

  // Enable all motors (optional)
  digitalWrite(X_EN_PIN, LOW);  // LOW to enable
  digitalWrite(Y_EN_PIN, LOW);  // LOW to enable
  digitalWrite(Z_EN_PIN, LOW);  // LOW to enable

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

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // If in closed-loop mode, perform Z-axis control
  if (closedLoopMode) {
    closedLoopControlZ();
  }
}

void printHelp() {
  Serial.println("\n=== 3-Axis Control Instructions ===");
  Serial.println("Commands:");
  Serial.println("X100 : Move X axis 100 steps right");
  Serial.println("X-100: Move X axis 100 steps left");
  Serial.println("Y100 : Move Y axis 100 steps forward");
  Serial.println("Y-100: Move Y axis 100 steps backward");
  Serial.println("Z100 : Move Z axis 100 steps up");
  Serial.println("Z-100: Move Z axis 100 steps down");
  // Serial.println("ML    : Select Open Loop Mode");
  Serial.println("CLZ  : Enter closed-loop control mode for Z-axis");
  Serial.println("S     : Stop all movement");
  Serial.println("H     : Show this help message");
  Serial.println("===============================");
}

// //-------------------------------------------------------------------------------------------------------------------------

// void loop() {
//   // Check if data is available to read
//   if (Serial.available() > 0) {
//     // Read the input string
//     String input = Serial.readStringUntil('\n');

//     // Check if the input is to stop the motor
//     if (input.equalsIgnoreCase("S")) {
//       running = false;
//       Serial.println("Motor stopped.");
//     } else {
//       // Extract the speed value from the input string
//       int rpm = input.substring(0, input.length() - 1).toInt();

//       // Constrain the speed to be between 1 and 1000 RPM
//       rpm = constrain(rpm, 1, 1000);

//       // Convert RPM to delay in microseconds between steps
//       speed = 60000000 / (rpm * stepsPerRevolution); // Correct calculation

//       // Extract the direction from the input string
//       direction = input.charAt(input.length() - 1);

//       // Set the direction
//       digitalWrite(dirPin, (direction == 'U') ? HIGH : LOW); // 'U' for forward, 'D' for backward

//       // Set the running state to true
//       running = true;

//       // Print the speed and direction to the Serial Monitor
//       Serial.print("Motor speed set to: ");
//       Serial.print(rpm);
//       Serial.print(" RPM, direction: ");
//       Serial.println(direction == 'U' ? "Forward" : "Backward");
//     }
//   }

//   // If the motor is running, generate steps
//   if (running) {
//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(speed / 50); // Half the delay for the high state
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(speed / 50); // Half the delay for the low state
//   }
// }

// //------------------------------------------------------------------------------------------------------------------

void processCommand(String command) {
  command.trim();  // Remove any whitespace

  // Check for help command
  if (command == "H" || command == "h") {
    printHelp();
    return;
  }

  // Check for stop command
  if (command == "S" || command == "s") {
    isMoving = false;
    closedLoopMode = false;
    Serial.println("Stopped all movement and exited closed-loop mode.");
    return;
  }

  // Check for closed-loop Z-axis mode
  if (command == "CLZ" || command == "clz") {
    closedLoopMode = true;
    Serial.println("Entered closed-loop control mode for Z-axis.");
    return;
  }

  // Process open-loop movement commands
  if (command.length() > 1) {
    char axis = command.charAt(0);
    int steps = command.substring(1).toInt();

    // Select pins based on axis
    int stepPin, dirPin, enPin;
    switch (axis) {
      case 'X':
      case 'x':
        stepPin = X_STEP_PIN;
        dirPin = X_DIR_PIN;
        enPin = X_EN_PIN;
        xCoord += steps;
        break;
      case 'Y':
      case 'y':
        stepPin = Y_STEP_PIN;
        dirPin = Y_DIR_PIN;
        enPin = Y_EN_PIN;
        yCoord += steps;
        break;
      case 'Z':
      case 'z':
        stepPin = Z_STEP_PIN;
        dirPin = Z_DIR_PIN;
        enPin = Z_EN_PIN;
        zCoord += steps;
        break;
      default:
        Serial.println("Invalid axis. Use X, Y, or Z.");
        return;
    }

    // Set direction based on positive or negative steps
    digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
    steps = abs(steps);  // Convert to positive for movement

    // Enable motor
    digitalWrite(enPin, LOW);  // LOW = enabled

    // Move the specified number of steps
    Serial.print("Moving ");
    Serial.print(axis);
    Serial.print(" axis ");
    Serial.print(steps);
    Serial.println(" steps");

    isMoving = true;
    for (int i = 0; i < steps && isMoving; i++) {
      digitalWrite(stepPin, HIGH);  // Pulse HIGH
      delayMicroseconds(STEP_DELAY);
      digitalWrite(stepPin, LOW);   // Pulse LOW
      delayMicroseconds(STEP_DELAY);
    }

    Serial.println("Movement complete");

    // Disable motor (optional)
    digitalWrite(enPin, HIGH);  // HIGH = disabled

    // Update LCD
    updateLCD();
  }
}

void readVoltage() {
  int A0Value = analogRead(voltagePin);
  float voltage_sensed = A0Value * (arduinoVCC / (float)inputResolution);
  voltage = 10 * voltage_sensed * (1 + ((float)ValueR2 / (float)ValueR1));
}

void closedLoopControlZ() {
  readVoltage();

  if (voltage >= 20.0) {
    // Move Z-axis downward until voltage drops
    digitalWrite(Z_DIR_PIN, LOW);  // Downward direction
    while (voltage >= 20.0 && closedLoopMode) {
      digitalWrite(Z_STEP_PIN, HIGH);
      delayMicroseconds(STEP_DELAY);
      digitalWrite(Z_STEP_PIN, LOW);
      delayMicroseconds(STEP_DELAY);
      readVoltage();
      zCoord--;
      updateLCD();
    }
  } else {
    // Move Z-axis upward until voltage rises
    digitalWrite(Z_DIR_PIN, HIGH);  // Upward direction
    while (voltage < 20.0 && closedLoopMode) {
      digitalWrite(Z_STEP_PIN, HIGH);
      delayMicroseconds(STEP_DELAY);
      digitalWrite(Z_STEP_PIN, LOW);
      delayMicroseconds(STEP_DELAY);
      readVoltage();
      zCoord++;
      updateLCD();
    }
  }

  Serial.println("Closed-loop control complete for current condition.");
}

void updateLCD() {
  // Clear the previous values
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.print(xCoord);
  lcd.print(" Y:");
  lcd.print(yCoord);
  lcd.print(" Z:");
  lcd.print(zCoord);
  lcd.print("   ");  // Overwrite any leftover characters

  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(voltage, 2);  // Print voltage with 2 decimal places
  // lcd.print(" Mode:");
  // lcd.print(currentMode == OPEN_LOOP ? "Open" : "ClsdZ");
}
