#ifndef CONFIG_H  // Prevent multiple inclusions of this header file
#define CONFIG_H  // Define the header guard

// Include necessary headers for driver object types
#include <Adafruit_PWMServoDriver.h>  // Library for PCA9685 PWM driver
#include <Servo.h>                    // Library for servo motor control

// PCA9685 PWM channels for motor control
#define PWM_LF 0  // Channel for left front motor
#define PWM_LR 1  // Channel for left rear motor (not used in current code)
#define PWM_RF 2  // Channel for right front motor
#define PWM_RR 3  // Channel for right rear motor (not used in current code)

// L298N motor driver control pins
#define in1_left 13   // Input 1 for left motor direction
#define in2_left 12   // Input 2 for left motor direction
#define in1_right 7   // Input 1 for right motor direction
#define in2_right 8   // Input 2 for right motor direction

// Collection mechanism control pins
#define in1_collect 2  // Input 1 for collection motor direction
#define in2_collect 4  // Input 2 for collection motor direction

// Servo motor pin
#define Servo_pin 6  // Pin connected to the servo motor

// Buzzer pin
#define ring 9  // Pin connected to the buzzer for alerts

// Infrared sensor pin for interrupt
#define Sensor_input 3  // Pin for IR sensor (change if pin conflict occurs)

// Speed duty cycles for PWM control
#define SPEED_FULL 4095  // Full speed duty cycle (max: 4095)
#define SPEED_HALF 2048  // Half speed duty cycle
#define SPEED_LOW  1024  // Low speed duty cycle
#define SPEED_STOP 0     // Stop (no speed) duty cycle

// Servo control angles
extern int angle_open;    // Angle to open the servo gate
extern int angle_close;   // Angle to close the servo gate

// Interrupt variables
extern volatile int ball_count;   // Counter for balls detected (volatile for interrupt)
extern volatile bool unload_trig; // Trigger for unloading (not used in current code)

// Communication variables
extern char char_recv;       // Character received from Bluetooth
extern String serialCommand; // Command string received via Serial

// Auto mode communication variables
extern char mode;      // Mode character from Serial command
extern char command;   // Command character from Serial command
extern String response; // Response string (not used in current code)

// Auto mode flag
extern bool isAutoMode; // Flag to indicate if in auto mode

// Driver objects
extern Adafruit_PWMServoDriver pwm;  // PWM driver object for motor control
extern Servo myServo;                // Servo object for gate control

// Global timing and state variables
extern unsigned long startTime;   // Start time for timing movements
extern unsigned long elapsedTime; // Elapsed time for movements
extern bool first_time;           // Flag for first loop iteration

#endif  // End of header guard