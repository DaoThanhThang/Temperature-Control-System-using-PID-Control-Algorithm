/**
 * @file main.cpp
 * @brief Tests for Simple Temperature Controller using PID.
 * 
 * This set of test cases aims to verify the functionality of the Simple Temperature Controller using PID.
 * 
 * @author Dao Thanh Thang
 * @email thang.techcareers@gmail.com
 */

#include <Arduino.h>
#include "PID.h"
#include <unity.h>

// Constants
#define TEMP_MIN 16    ///< Minimum temperature in Celsius
#define TEMP_MAX 30    ///< Maximum temperature in Celsius
#define PIN_SENSOR A0  ///< Pin for temperature sensor
#define PIN_ADJUST A1  ///< Pin for adjustment
#define PIN_ACTUATOR 9 ///< Pin for actuator control (PWM)
#define DIRECT 1       ///< Forward control direction
#define REVERSE -1     ///< Reverse control direction
#define AUTOMATIC 1    ///< Automatic mode for PID controller
#define MANUAL 0       ///< Manual mode for PID controller

// Global variables
double setpoint = 25.0; ///< Desired temperature
double input, output;   ///< Temperature input and output
double Kp = 1.2, Ki = 1, Kd = 0.1; ///< PID tuning parameters
int pwmValue; ///< PWM value

// Create PID controller object
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/**
 * @brief Test case to check if the PID controller is properly initialized.
 * 
 * The test checks if the PID controller object is created and initialized correctly.
 */
void testPIDInitialization() {
  TEST_ASSERT_EQUAL_DOUBLE(Kp, pid.GetKp());
  TEST_ASSERT_EQUAL_DOUBLE(Ki, pid.GetKi());
  TEST_ASSERT_EQUAL_DOUBLE(Kd, pid.GetKd());
}

/**
 * @brief Test case to check if temperature readings are within the valid range.
 * 
 * The test reads temperature from the sensor and checks if it falls within the valid range (16-30°C).
 */
void testTemperatureRange() {
  input = analogRead(PIN_SENSOR);
  input = input * 0.48828125; // Convert ADC value to Celsius
  TEST_ASSERT_TRUE(input >= TEMP_MIN && input <= TEMP_MAX);
}

/**
 * @brief Test case to check if the actuator control is functioning correctly.
 * 
 * The test sets the temperature input above the setpoint and verifies if the actuator is opened.
 */
void testActuatorControl() {
    input = setpoint + 1; // Set temperature above setpoint
    TEST_ASSERT_EQUAL_INT(HIGH, digitalRead(PIN_ACTUATOR)); // Actuator should be open
}

/**
 * @brief Test case to check if the setpoint update function works as expected.
 * 
 * The test sends a setpoint update command via serial communication and verifies if the setpoint is correctly updated.
 */
void testSetpointUpdate() {
  Serial.println("SET:28"); // Send setpoint update command
  delay(100); // Allow time for processing
  TEST_ASSERT_EQUAL_DOUBLE(28.0, setpoint); // Check if setpoint is updated
}
void runTests() {
  UNITY_BEGIN();
  RUN_TEST(testPIDInitialization);
  RUN_TEST(testTemperatureRange);
  RUN_TEST(testActuatorControl);
  RUN_TEST(testSetpointUpdate);
  UNITY_END();
}

/**
 * @brief Main function to initialize setup and run tests.
 */
void setup() {
    delay(2000);
  Serial.begin(9600);
  pid.SetMode(AUTOMATIC);
  runTests();
}
void loop(){

}
