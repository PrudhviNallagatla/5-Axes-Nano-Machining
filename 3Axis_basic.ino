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

// Motor parameters
const int STEP_DELAY = 500;  // Delay between steps (microseconds) - adjust for speed
bool isMoving = false;       // Flag to track if any motor is moving

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

  // Enable all motors (optional)
  digitalWrite(X_EN_PIN, LOW);  // LOW to enable
  digitalWrite(Y_EN_PIN, LOW);  // LOW to enable
  digitalWrite(Z_EN_PIN, LOW);  // LOW to enable

  // Start serial communication
  Serial.begin(9600);

  // Print instructions
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
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
  Serial.println("S     : Stop all movement");
  Serial.println("H     : Show this help message");
  Serial.println("===============================");
}

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
    Serial.println("Stopped all movement");
    return;
  }

  // Process movement commands
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
        break;
      case 'Y':
      case 'y':
        stepPin = Y_STEP_PIN;
        dirPin = Y_DIR_PIN;
        enPin = Y_EN_PIN;
        break;
      case 'Z':
      case 'z':
        stepPin = Z_STEP_PIN;
        dirPin = Z_DIR_PIN;
        enPin = Z_EN_PIN;
        break;
      default:
        Serial.println("Invalid axis. Use X, Y, or Z");
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
  }
}