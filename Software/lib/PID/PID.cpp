#include "PID.h"

/**
 * @brief Constructor for PID controller.
 * @param input Pointer to input value.
 * @param output Pointer to output value.
 * @param setpoint Pointer to setpoint value.
 * @param Kp Proportional term.
 * @param Ki Integral term.
 * @param Kd Derivative term.
 * @param controllerDirection Direction of controller (+1 or -1).
 */
PID::PID(double* input, double* output, double* setpoint, double Kp, double Ki, double Kd, int controllerDirection) {
  myInput = input;
  myOutput = output;
  mySetpoint = setpoint;
  inAuto = false;

  SetTunings(Kp, Ki, Kd);
  SetSampleTime(1000); // Default sample time: 1 second
  SetOutputLimits(0, 255); // Default output limits: 0 to 255
  SetMode(1); // Default mode: PID on
  controllerDirection = controllerDirection;
}

/**
 * @brief Compute PID output.
 */
void PID::Compute() {
  if (!inAuto) return;
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (timeChange >= sampleTime) {
    double input = *myInput;
    double error = *mySetpoint - input;

    ITerm += (ki * error);
    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;

    double dInput = (input - lastInput);

    double output = kp * error + ITerm - kd * dInput;
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;

    *myOutput = output;

    lastInput = input;
    lastTime = now;
  }
}

/**
 * @brief Set PID tuning parameters.
 * @param Kp Proportional term.
 * @param Ki Integral term.
 * @param Kd Derivative term.
 */
void PID::SetTunings(double Kp, double Ki, double Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  kp = Kp;
  ki = Ki * sampleTime / 1000.0;
  kd = Kd / sampleTime * 1000.0;

  if (controllerDirection == -1) {
    kp = 0 - kp;
    ki = 0 - ki;
    kd = 0 - kd;
  }
}

/**
 * @brief Set the PID sample time (in milliseconds).
 * @param sampleTime Sample time in milliseconds.
 */
void PID::SetSampleTime(int newSampleTime) {
  if (newSampleTime > 0) {
    double ratio = (double)newSampleTime / (double)sampleTime;
    ki *= ratio;
    kd /= ratio;
    sampleTime = (unsigned long)newSampleTime;
  }
}

/**
 * @brief Set PID output limits.
 * @param outMin Minimum output value.
 * @param outMax Maximum output value.
 */
void PID::SetOutputLimits(double outMin, double outMax) {
  if (outMin >= outMax) return;
  this->outMin = outMin;
  this->outMax = outMax;

  if (inAuto) {
    if (*myOutput > outMax) *myOutput = outMax;
    else if (*myOutput < outMin) *myOutput = outMin;

    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;
  }
}

/**
 * @brief Set PID mode (1 = PID on, 0 = PID off).
 * @param mode PID mode.
 */
void PID::SetMode(int mode) {
  bool newAuto = (mode == 1);
  if (newAuto && !inAuto) {
    Initialize();
  }
  inAuto = newAuto;
}

/**
 * @brief Initialize PID controller.
 */
void PID::Initialize() {
  lastInput = *myInput;
  ITerm = *myOutput;
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
}
