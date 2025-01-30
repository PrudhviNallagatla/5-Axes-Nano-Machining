// Define the pins used
const int stepPin = 3;     // Pin to send steps
const int dirPin = 2;      // Pin to set direction
const int voltagePin = A0; // Pin to read the voltage

// Define the number of steps per revolution for your motor
const int motorStepsPerRevolution = 200; // Motor steps per revolution (assuming standard motor)
const int microsteps = 32;               // Microsteps setting for TB6600
const int stepsPerRevolution = 6400;     // Total steps per revolution with microstepping

// Variables to hold user input for speed and directio
int speed = 0;        // Speed in microseconds between steps
char direction = 'F'; // 'F' for forward, 'B' for backward
bool running = false; // Motor running state

// Voltage variables
const float arduinoVCC = 5.01; // Your Arduino voltage
unsigned long ValueR1 = 98700;
unsigned long ValueR2 = 9870;
double Voltage_Source = 12;
const int analogPin = A0;         // the pin connecting the voltage
const int inputResolution = 1023; // works with most Arduino boards
float voltage;

void setup()
{
  // Set the pins as output
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(voltagePin, INPUT);

  // Initialize the serial port
  Serial.begin(9600);

  // Ask the user for operation mode
  Serial.println("Choose operation mode:");
  Serial.println("1. Open loop");
  Serial.println("2. Closed loop");
  Serial.println("Enter the number corresponding to your choice:");

  // Wait for user input
  while (!Serial.available())
  {
    // Wait for user input
  }

  // Read user choice
  int choice = Serial.parseInt();

  // Check user choice and proceed accordingly
  if (choice == 1)
  {
    // Open loop mode
    Serial.println("Open loop mode selected.");
    Serial.println("Enter speed in RPM (1-1000) followed by U for forward or D for backward, e.g., 100U:");
    Serial.println("Enter 'S' to stop the motor.");
  }
  else if (choice == 2)
  {
    // Closed loop mode
    Serial.println("Closed loop mode selected.");
    // Start the closed loop control
    closedLoopControl();
  }
  else
  {
    // Invalid choice
    Serial.println("Invalid choice. Restart the Arduino and select 1 or 2.");
    while (true)
    {
      // Do nothing, wait for reset
    }
  }
}

void loop()
{
  // Check if data is available to read
  if (Serial.available() > 0)
  {
    // Read the input string
    String input = Serial.readStringUntil('\n');

    // Check if the input is to stop the motor
    if (input.equalsIgnoreCase("S"))
    {
      running = false;
      Serial.println("Motor stopped.");
    }
    else
    {
      // Extract the speed value from the input string
      int rpm = input.substring(0, input.length() - 1).toInt();

      // Constrain the speed to be between 1 and 1000 RPM
      rpm = constrain(rpm, 1, 1000);

      // Convert RPM to delay in microseconds between steps
      speed = 60000000 / (rpm * stepsPerRevolution); // Correct calculation

      // Extract the direction from the input string
      direction = input.charAt(input.length() - 1);

      // Set the direction
      digitalWrite(dirPin, (direction == 'U') ? HIGH : LOW); // 'U' for forward, 'D' for backward

      // Set the running state to true
      running = true;

      // Print the speed and direction to the Serial Monitor
      Serial.print("Motor speed set to: ");
      Serial.print(rpm);
      Serial.print(" RPM, direction: ");
      Serial.println(direction == 'U' ? "Forward" : "Backward");
    }
  }

  // If the motor is running, generate steps
  if (running)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speed / 50); // Half the delay for the high state
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speed / 50); // Half the delay for the low state
  }
}

void readVoltage()
{
  int A0Value = analogRead(analogPin);
  float voltage_sensed = A0Value * (arduinoVCC / (float)inputResolution);
  voltage = 10 * voltage_sensed * (1 + ((float)ValueR2 / (float)ValueR1));
}

void closedLoopControl()
{
  bool stop = false;

  while (!stop)
  {
    if (Serial.available() > 0)
    {
      // Read the input string
      String input = Serial.readStringUntil('\n');

      // Check if the input is to stop the motor
      if (input.equalsIgnoreCase("S"))
      {
        stop = true;
        Serial.println("Motor stopped.");
      }
    }

    // Read the voltage value from the analog pi
    readVoltage();
    // Control the motor based on the voltage reading
    if (voltage >= 20.0)
    {
      // Move downwards until voltage drops
      digitalWrite(dirPin, LOW); // Set direction to down
      while (voltage >= 20.0)
      {
        if (Serial.available() > 0)
        {
          String input = Serial.readStringUntil('\n');
          if (input.equalsIgnoreCase("S"))
          {
            stop = true;
            Serial.println("Motor stopped.");
            break;
          }
        }
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500); // Adjust this delay for speed control
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500); // Adjust this delay for speed control
        readVoltage();
      }
    }
    else
    {
      // Move upwards slowly
      digitalWrite(dirPin, HIGH); // Set direction to up
      while (voltage < 20.0)
      {
        if (Serial.available() > 0)
        {
          String input = Serial.readStringUntil('\n');
          if (input.equalsIgnoreCase("S"))
          {
            stop = true;
            Serial.println("Motor stopped.");
            break;
          }
        }
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000); // Adjust this delay for slower speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000); // Adjust this delay for slower speed
        readVoltage();
      }
    }
  }
}
