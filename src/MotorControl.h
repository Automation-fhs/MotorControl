#include <Arduino.h>

class MotorControl
{
public:
    MotorControl(int rated_V, int enc_pulse_per_phase, int gear_ratio, float time_interval, int Controller = 1, int pwm_res = 255);
    void setpwm_res(int pwm_res);
    int CtrlSignl(float setpoint, long curPulse, int unit = 1); // Unit: 1 for Degree; 2 for Radian

private:
    float _getDeg(long curPulse);
    float _getRad(long curPulse);
    int _rated_V;
    int _encPPP;
    int _gear_ratio;
    int _Controller;
    int _pwm_res;

    float _prev_err = 0;
    float _integral;
    uint32_t _integralTimer = 0;
    float _time_interval;
    float _prev_setpoint = 0;
    bool _new_setpoint;
    bool _ovsh = false;
};