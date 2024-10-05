#include "./RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(String enc_type = "AB", bool NR = true)
{

    this->_enc_type = enc_type;
    this->_NR = NR;
}

void RotaryEncoder::encInit(int cur_A, int cur_B)
{
    if (this->_NR)
    {
        this->_encNRVal = ((cur_A << 1) | cur_B | this->_encNRVal) & 0x0f;
    }
    else
    {
        this->_prev_A = cur_A;
        this->_prev_B = cur_B;
    }
}

void RotaryEncoder::upd_Pulse(int cur_A, int cur_B)
{
    if (this->_NR)
    {
        this->_encNRVal = this->_encNRVal << 2;
        this->_encNRVal = ((cur_A << 1) | cur_B | this->_encNRVal) & 0x0f;
        this->_cur_pulse -= Enc_NR[this->_encNRVal];
    }
    else
    {
        if (cur_A == _prev_A)
        {
            if (cur_B != _prev_B)
            {
                if (cur_A != cur_B)
                {
                    _cur_pulse += 1;
                }
                else
                {
                    _cur_pulse -= 1;
                }
            }
        }
        else
        {
            if (cur_B == _prev_B)
            {
                if (cur_A != cur_B)
                {
                    _cur_pulse -= 1;
                }
                else
                {
                    _cur_pulse += 1;
                }
            }
        }
        _prev_A = cur_A;
        _prev_B = cur_B;
    }
}

long RotaryEncoder::getCurPulse()
{
    return this->_cur_pulse;
}

void RotaryEncoder::setCurPulse(long pulse)
{
    this->_cur_pulse = pulse;
}