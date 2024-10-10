#include <math.h>
#include "./MotorControl.h"
#include "./Encoder/RotaryEncoder.h"
// #include "./Fuzzy/Fuzzy.h"
#include "./FuzzyPID/FuzzyPID.h"
#include "./PID/PID.h"
#include "./ControllerConfig.h"
#include <Arduino.h>

PID myPID = PID(PIDfactor);
Fuzzy myFuzzy = Fuzzy(err, derr, 7, FuzzyRules);
FuzzyPID myfPID = FuzzyPID(PIDfactor, FErr, dFErr, 7, PRules, IRules, DRules);

MotorControl::MotorControl(int rated_V, int enc_pulse_per_phase, int gear_ratio, float time_interval, int Controller = 1, int pwm_res = 255)
{
    this->_rated_V = rated_V;
    this->_encPPP = enc_pulse_per_phase;
    this->_gear_ratio = gear_ratio;
    this->_time_interval = time_interval;
    this->_Controller = Controller;
    this->_pwm_res = pwm_res;
}

float MotorControl::_getDeg(long pulse)
{
    return float(pulse) / (2 * this->_encPPP * this->_gear_ratio) * 180;
}

float MotorControl::_getRad(long pulse)
{
    return float(pulse) / (2 * this->_encPPP * this->_gear_ratio) * M_PI;
}

int MotorControl::CtrlSignl(float setpoint, long curPulse, int unit = 1)
{
    float _setpoint;
    int result;
    if (setpoint != this->_prev_setpoint)
    {
        this->_integral = 0;
        this->_prev_setpoint = setpoint;
        this->_new_setpoint = true;
    }

    // Convert setpoint Unit to Radian
    if (unit == 1)
        _setpoint = setpoint;
    else
        _setpoint = setpoint * 180 / M_PI;

    // Calculate error, change of error and integral
    float err = _setpoint - this->_getDeg(curPulse);
    this->_integral += err * this->_time_interval;
    float derr = (err - this->_prev_err) / this->_time_interval;

    // Reset derr and integral due to setpoint changed
    if (this->_new_setpoint)
    {
        derr = 0;
        this->_integral = 0;
        this->_ovsh = true; 
        this->_new_setpoint = false;
    }
    // Serial.println(this->_getDeg(curPulse));
    this->_prev_err = err;

    // Remove integral to prevent overshoot
    if (this->_ovsh && abs(err) <= 5)
    {
        this->_integral = 0;
        this->_ovsh = false;
    }

    if (this->_Controller == 1)
    { // =================== PID Controller ===================
        result = (int)(myPID.Result(err, derr, this->_integral) * float(this->_pwm_res) / this->_rated_V);
    }
    else if (this->_Controller == 2)
    { // ------------------- Fuzzy Controller -------------------
        result = -(int)(myFuzzy.Result(err, derr));
        // Serial.println(this->_getDeg(curPulse));
    }
    else if (this->_Controller == 3)
    { // ------------------- FuzzyPID Controller -------------------
        result = (int)(myfPID.Result(err, derr, this->_integral) * float(this->_pwm_res) / this->_rated_V);
        // Serial.println(this->_getDeg(curPulse));
    }
    else
    {
        result = 0;
        //  Serial.println("Invalid controller");
    }

    if (result > this->_pwm_res)
        return this->_pwm_res;
    else if (result < -this->_pwm_res)
        return -this->_pwm_res;
    else
        return result;
}
