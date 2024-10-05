#include <Arduino.h>

class RotaryEncoder
{
public:
    RotaryEncoder(String enc_type = "AB", bool NR = true);
    void encInit(int cur_A, int cur_B);
    void upd_Pulse(int cur_A, int cur_B);
    long getCurPulse();
    void setCurPulse(long pulse);

private:
    String _enc_type;
    char _encNRVal;
    bool _NR;
    long _prev_A;
    long _prev_B;
    long _cur_pulse;
    int Enc_NR[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
};