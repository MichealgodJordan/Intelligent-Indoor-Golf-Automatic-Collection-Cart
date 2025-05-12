#include <Servo.h>           // Include Servo library for servo control
#include <Wire.h>            // Include Wire library for I2C communication
#include <SoftwareSerial.h>  // Include SoftwareSerial for Bluetooth communication
#include "config.h"          // Include custom configuration file

// Define global variables from config.h
int angle_open = 130;              // Set servo open angle to 130 degrees
int angle_close = 25;              // Set servo close angle to 25 degrees
volatile int ball_count = 0;       // Initialize ball counter to 0 (volatile for interrupt)
volatile bool unload_trig = false; // Initialize unload trigger to false (not used)
char char_recv;                    // Variable to store received Bluetooth character
String serialCommand;              // String to store Serial command
char mode;                         // Variable for mode from Serial command
char command;                      // Variable for command from Serial command
String response;                   // String for response (not used)
bool isAutoMode = false;           // Initialize auto mode flag to false
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // Initialize PWM driver object
Servo myServo = Servo();           // Initialize servo object
unsigned long startTime = 0;       // Initialize start time to 0
unsigned long elapsedTime = 0;     // Initialize elapsed time to 0
bool first_time = true;            // Set first_time flag to true for initial run

// Define SoftwareSerial object for Bluetooth
SoftwareSerial bluetooth(10, 11);  // RX pin 10, TX pin 11 for HC-05 Bluetooth module

void setup() {  // Setup function runs once at startup
  Serial.begin(9600);  // Start Serial communication at 9600 baud for debugging
  bluetooth.begin(9600);  // Start Bluetooth Serial at 9600 baud for HC-05
  pinMode(in1_left, OUTPUT);   // Set left motor in1 pin as output
  pinMode(in2_left, OUTPUT);   // Set left motor in2 pin as output
  pinMode(in1_right, OUTPUT);  // Set right motor in1 pin as output
  pinMode(in2_right, OUTPUT);  // Set right motor in2 pin as output
  pinMode(Sensor_input, INPUT);  // Set IR sensor pin as input
  pinMode(ring, OUTPUT);       // Set buzzer pin as output
  pinMode(in1_collect, OUTPUT);  // Set collection motor in1 pin as output
  pinMode(in2_collect, OUTPUT);  // Set collection motor in2 pin as output
  attachInterrupt(digitalPinToInterrupt(Sensor_input), ball_count_add, FALLING);  // Attach interrupt to IR sensor on falling edge
  myServo.attach(Servo_pin);   // Attach servo to its designated pin
  pwm.begin();                 // Initialize PWM driver
  pwm.setPWMFreq(1000);        // Set PWM frequency to 1000 Hz
  delay(10);                   // Wait 10ms for PWM initialization
  bluetooth.println("bluetooth started: default manual mode");  // Send startup message via Bluetooth
}

void loop() {  // Main loop function runs continuously
  if (first_time) {         // Check if this is the first iteration
    ball_count = 0;         // Reset ball count to 0 on first run
    first_time = false;     // Set first_time flag to false after reset
  }
  if (isAutoMode) {         // If in auto mode
    autoMode();             // Call auto mode function
  } else {                  // If not in auto mode
    manualMode();           // Call manual mode function
  }
}

void manualMode() {  // Function for manual control via Bluetooth
  unload_2();              // Close the servo gate
  bluetooth.println("enter manual mode (bluetooth)");  // Notify entry to manual mode
  bluetooth.setTimeout(10);  // Set Bluetooth timeout to 10ms to avoid blocking
  Serial.setTimeout(50);     // Set Serial timeout to 50ms for command reading
  while (true) {             // Infinite loop for manual mode
    if (ball_count >= 10) {  // Vital: Check if 10 balls are collected
      stop();                // Stop all motors
      while (true) {         // Loop until cleared
        digitalWrite(ring, HIGH);  // Turn buzzer on
        delay(500);           // Sound for 500ms
        digitalWrite(ring, LOW);   // Turn buzzer off
        delay(500);           // Silent for 500ms
        if (bluetooth.available()) {  // Check for Bluetooth data
          char btChar = bluetooth.read();  // Read Bluetooth character
          if (btChar == 'C') {  // If 'C' received
            ball_count = 0;     // Reset ball count
            break;              // Exit buzzer loop
          }
        }
      }
    }
    if (Serial.available()) {  // Check for Serial data
      serialCommand = Serial.readStringUntil('@');  // Read command until '@'
      serialCommand.trim();    // Remove whitespace
      if (serialCommand == "w") {  // If command is "w"
        Serial.println("w_a");  // Send acknowledgment
      }
    }
    if (bluetooth.available()) {  // Vital: Check for Bluetooth commands
      char_recv = bluetooth.read();  // Read received character
      bluetooth.print("received bluetooth command: ");  // Echo command
      bluetooth.println(char_recv);  // Print command
      switch (char_recv) {   // Process command
        case 'F': forward(); collection_rotate(); break;  // Move forward and collect
        case 'B': backward(); break;  // Move backward
        case 'L': turnLeft(); break;  // Turn left
        case 'R': turnRight(); break; // Turn right
        case 'S': stop(); break;      // Stop movement
        case 'U': unload_1(); break;  // Open servo gate
        case 'u': unload_2(); break;  // Close servo gate
        case 'Q': unloading(); break; // Start unloading sequence
        case 'A':            // Vital: Switch to auto mode
          bluetooth.println("switch to auto mode");  // Notify mode switch
          Serial.println("M_start");  // Send start signal
          stop();            // Stop all motors
          delay(1000);       // Wait 1 second
          isAutoMode = true; // Set auto mode flag
          return;            // Exit function
      }
    }
  }
}

void autoMode() {  // Function for autonomous control
  unload_2();              // Close the servo gate
  bluetooth.setTimeout(5); // Set Bluetooth timeout to 5ms
  Serial.setTimeout(10);   // Set Serial timeout to 10ms
  while (true) {           // Infinite loop for auto mode
    Serial.println(ball_count);  // Print current ball count
    if (ball_count >= 10) {  // Vital: Check if 10 balls collected
      stop();                // Stop all motors
      while (true) {         // Loop until cleared
        digitalWrite(ring, HIGH);  // Turn buzzer on
        delay(500);           // Sound for 500ms
        digitalWrite(ring, LOW);   // Turn buzzer off
        delay(500);           // Silent for 500ms
        if (bluetooth.available()) {  // Check Bluetooth
          char btChar = bluetooth.read();  // Read character
          if (btChar == 'C') {  // If 'C' received
            isAutoMode = false; // Exit auto mode
            ball_count = 0;     // Reset ball count
            Serial.println("M_stop");  // Send stop signal
            delay(1000);        // Wait 1 second
            return;             // Exit function
          }
        }
      }
    }
    if (bluetooth.available()) {  // Check for mode switch
      char btChar = bluetooth.read();  // Read character
      if (btChar == 'M') {  // If 'M' received
        isAutoMode = false; // Exit auto mode
        Serial.println("M_stop");  // Send stop signal
        stop();            // Stop motors
        delay(1000);       // Wait 1 second
        return;            // Exit function
      }
    }
    if (Serial.available()) {  // Vital: Process Serial commands
      serialCommand = Serial.readStringUntil('@');  // Read until '@'
      serialCommand.trim();    // Remove whitespace
      mode = serialCommand.charAt(0);  // Get mode character
      command = serialCommand.charAt(2);  // Get command character
      if (mode == 'P') {       // Precise mode
        if (command == 'F') {  // Forward command
          startTime = millis();  // Record start time
          forward();          // Move forward
          collection_rotate();  // Start collection
        } else if (command == 'L') {  // Left turn
          turnLeft();         // Turn left
          delay(150);         // Turn for 150ms
        } else if (command == 'R') {  // Right turn
          turnRight();        // Turn right
          delay(150);         // Turn for 150ms
        } else {              // Unknown command
          bluetooth.println("unknown command type");  // Notify error
        }
        if (command != 'F') {  // If not forward
          stop();             // Stop motors
          delay(1000);        // Wait 1 second
          Serial.println("p");  // Send completion signal
        } else {              // If forward
          while (true) {      // Loop until sensor triggered
            if (digitalRead(Sensor_input) == 0) {  // Sensor detects object
              stop();         // Stop motors
              elapsedTime = millis() - startTime;  // Calculate time
              delay(500);     // Wait 500ms
              backward();     // Move backward
              delay(elapsedTime);  // For same duration
              stop();         // Stop motors
              delay(500);     // Wait 500ms
              Serial.println("p_f");  // Send completion signal
              break;          // Exit loop
            }
            if (bluetooth.available()) {  // Check Bluetooth
              char btChar = bluetooth.read();  // Read character
              if (btChar == 'M') {  // If 'M' received
                isAutoMode = false; // Exit auto mode
                Serial.println("M_stop");  // Send stop signal
                stop();         // Stop motors
                delay(1250);    // Wait 1.25 seconds
                return;         // Exit function
              }
            }
          }
        }
      } else if (mode == 'C') {  // Coarse mode
        if (command == 'F') {    // Forward command
          forward();            // Move forward
          delay(1000);          // For 1 second
        } else if (command == 'L') {  // Left turn
          turnLeft();           // Turn left
          delay(1500);          // For 1.5 seconds
        } else if (command == 'R') {  // Right turn
          turnRight();          // Turn right
          delay(1500);          // For 1.5 seconds
        } else {                // Unknown command
          bluetooth.println("unknown command type");  // Notify error
        }
        stop();                 // Stop motors
        Serial.println("c");    // Send completion signal
      }
    }
  }
}

// Motion control functions
void forward() {  // Move cart forward
  digitalWrite(in1_left, LOW);    // Set left motor direction forward
  digitalWrite(in2_left, HIGH);   // Set left motor direction forward
  digitalWrite(in1_right, HIGH);  // Set right motor direction forward
  digitalWrite(in2_right, LOW);   // Set right motor direction forward
  pwm.setPWM(PWM_LF, 0, SPEED_FULL);  // Set left front motor to full speed
  pwm.setPWM(PWM_RF, 0, SPEED_FULL);  // Set right front motor to full speed
}

void backward() {  // Move cart backward
  digitalWrite(in1_left, HIGH);   // Set left motor direction backward
  digitalWrite(in2_left, LOW);    // Set left motor direction backward
  digitalWrite(in1_right, LOW);   // Set right motor direction backward
  digitalWrite(in2_right, HIGH);  // Set right motor direction backward
  pwm.setPWM(PWM_LF, 0, SPEED_FULL);  // Set left front motor to full speed
  pwm.setPWM(PWM_RF, 0, SPEED_FULL);  // Set right front motor to full speed
}

void turnLeft() {  // Turn cart left
  digitalWrite(in1_left, HIGH);   // Set left motor direction backward
  digitalWrite(in2_left, LOW);    // Set left motor direction backward
  digitalWrite(in1_right, HIGH);  // Set right motor direction forward
  digitalWrite(in2_right, LOW);   // Set right motor direction forward
  pwm.setPWM(PWM_LF, 0, SPEED_HALF);  // Set left front motor to half speed
  pwm.setPWM(PWM_RF, 0, SPEED_FULL);  // Set right front motor to full speed
}

void turnRight() {  // Turn cart right
  digitalWrite(in1_left, LOW);    // Set left motor direction forward
  digitalWrite(in2_left, HIGH);   // Set left motor direction forward
  digitalWrite(in1_right, LOW);   // Set right motor direction backward
  digitalWrite(in2_right, HIGH);  // Set right motor direction backward
  pwm.setPWM(PWM_LF, 0, SPEED_FULL);  // Set left front motor to full speed
  pwm.setPWM(PWM_RF, 0, SPEED_HALF);  // Set right front motor to half speed
}

void stop() {  // Stop all motors
  digitalWrite(in1_left, LOW);    // Turn off left motor input 1
  digitalWrite(in2_left, LOW);    // Turn off left motor input 2
  digitalWrite(in1_right, LOW);   // Turn off right motor input 1
  digitalWrite(in2_right, LOW);   // Turn off right motor input 2
  digitalWrite(in1_collect, LOW); // Turn off collection motor input 1
  digitalWrite(in2_collect, LOW); // Turn off collection motor input 2
  pwm.setPWM(PWM_LF, 0, SPEED_STOP);  // Stop left front motor
  pwm.setPWM(PWM_RF, 0, SPEED_STOP);  // Stop right front motor
}

void unload_1() {  // Open servo gate
  myServo.write(angle_open);  // Set servo to open angle
}

void unload_2() {  // Close servo gate
  myServo.write(angle_close);  // Set servo to close angle
}

void collection_rotate() {  // Rotate collection mechanism
  digitalWrite(in1_collect, LOW);   // Set collection motor direction
  digitalWrite(in2_collect, HIGH);  // Set collection motor direction
}

void unloading() {  // Perform unloading sequence
  unload_1();          // Open servo gate
  while (true) {       // Loop until stopped
    forward();         // Move forward
    delay(200);        // For 200ms
    backward();        // Move backward
    delay(200);        // For 200ms
    if (bluetooth.available()) {  // Check Bluetooth
      char btChar = bluetooth.read();  // Read character
      if (btChar == 'D') {  // If 'D' received
        stop();           // Stop motors
        break;            // Exit loop
      }
    }
  }
  ball_count = 0;      // Reset ball count
  unload_2();          // Close servo gate
}

void ball_count_add() {  // Interrupt function to increment ball count
  ball_count++;          // Increment ball counter
}