/**
 * @file main.cpp
 * @brief Simple Temperature Controller using PID with Arduino.
 * @author Dao Thanh Thang
 * @email thang.techcareers@gmail.com
 */

#include <Arduino.h>
#include "PID.h"

// Define constants
#define PIN_SENSOR A0
#define PIN_ADJUST A1
#define PIN_ACTUATOR 9 ///PWM 
#define DIRECT 1        ///< Forward control direction
#define REVERSE -1      ///< Reverse control direction
#define AUTOMATIC 1     ///< Automatic mode for PID controller
#define MANUAL 0        ///< Manual mode for PID controller

// Global variables
double setpoint = 25.0; ///< Desired temperature
double input, output;
double Kp = 1.2, Ki = 1, Kd = 0.1; ///< PID tuning parameters
int pwmValue;

// Create PID controller object
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/**
 * @brief Read data from serial port and update the setpoint temperature.
 * 
 * This function reads data from the serial port until a newline character is encountered.
 * It checks if the received data has the format "SET:+temperature".
 * If the format is correct, it extracts the temperature value from the string and updates the setpoint temperature accordingly.
 * If the temperature is below 16°C, the setpoint is set to 16°C. If it exceeds 30°C, the setpoint is set to 30°C.
 * It prints a confirmation message to the serial port after updating the setpoint.
 * If the received data does not match the expected format, it prints an error message.
 */
void readSerialAndUpdateSetpoint() {
  if (Serial.available() > 0) {
    // Read data from serial port until a newline character is encountered
    String inputString = Serial.readStringUntil('\n');
    // Check if the received data has the format "SET:+temperature"
    if (inputString.startsWith("SET:")) {
      // Extract the temperature value from the string, skipping the first 4 characters ("SET:")
      int newSetpoint = inputString.substring(4).toInt();
      // Check and update the setpoint temperature
      if (newSetpoint < 16) {
        setpoint = 16;
      } else if (newSetpoint > 30) {
        setpoint = 30;
      } else {
        setpoint = newSetpoint;
        Serial.println("MES:SUCCESS");
      }
    } else {
      // Print an error message if the received data does not match the expected format
      Serial.println("ERR:The temperature is only valid between 16-30 degrees Celsius");
    }
  }
}

/**
 * @brief Read temperature from the sensor and compute PID control.
 * 
 * This function reads the temperature from the sensor and converts it to Celsius.
 * It then computes the PID control based on the current temperature and setpoint temperature.
 */
void readTemperatureAndComputePID() {
  // Read temperature from the sensor
  input = analogRead(PIN_SENSOR);
  pwmValue = map(input, 0, 1023, 0, 255);
  input = input * 0.48828125; 
  
  // Compute PID control
  pid.Compute(); 
}

void setup() {
  Serial.begin(9600);
  pid.SetMode(AUTOMATIC); ///< Switch to automatic mode
}

/**
 * @brief Main loop function.
 * 
 * This function is the main loop of the program.
 * It sequentially executes the following tasks:
 * - Read data from serial port and update the setpoint temperature.
 * - Read temperature from the sensor and compute PID control.
 * - Control actuator based on temperature deviation from setpoint.
 * - Include a delay of 1 second between iterations to control the loop frequency.
 */
void loop() {
  // Read data from serial port and update the setpoint temperature
  readSerialAndUpdateSetpoint();
  // Read temperature from the sensor and compute PID control
  readTemperatureAndComputePID(); 
  // Control actuator based on temperature deviation from setpoint
  if (input > setpoint) {
    // Open the actuator (assuming it's a digital output)
    digitalWrite(PIN_ACTUATOR, HIGH);
  } else {
    // Close the actuator
    digitalWrite(PIN_ACTUATOR, LOW);
  }
  // Delay for 1 second
  delay(1000);
}
