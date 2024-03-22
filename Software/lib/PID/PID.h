#ifndef PID_h
#define PID_h

#include <Arduino.h>
/**
 * @file PID_v1.h
 * @class PID
 * @brief Class for implementing a PID controller.
 */
class PID {
  public:
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
    PID(double* input, double* output, double* setpoint, double Kp, double Ki, double Kd, int controllerDirection);

    // Functions
    void Compute(); /**< Compute PID output. */
    void SetTunings(double Kp, double Ki, double Kd); /**< Set PID tuning parameters. */
    void SetSampleTime(int sampleTime); /**< Set the PID sample time (in milliseconds). */
    void SetOutputLimits(double outMin, double outMax); /**< Set PID output limits. */
    void SetMode(int mode); /**< Set PID mode (1 = PID on, 0 = PID off). */
    void Initialize(); // Declaration of Initialize function
  private:
    double *myInput; /**< Pointer to input value. */
    double *myOutput; /**< Pointer to output value. */
    double *mySetpoint; /**< Pointer to setpoint value. */
    double kp; /**< Proportional term. */
    double ki; /**< Integral term. */
    double kd; /**< Derivative term. */
    int controllerDirection; /**< Direction of controller (+1 or -1). */
    unsigned long lastTime;
    double ITerm, lastInput;
    double outMin, outMax;
    bool inAuto;
    unsigned long sampleTime;
};

#endif
