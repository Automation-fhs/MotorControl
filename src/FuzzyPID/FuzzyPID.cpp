#include "./FuzzyPID.h"
#include <Arduino.h>
// #include "../Fuzzy/FuzzySet.h"

FuzzyPID::FuzzyPID(float PID[3], float err[7], float derr[7], int level, float PRules[7][7], float IRules[7][7], float DRules[7][7])
{
    this->_iPID[0] = PID[0];
    this->_iPID[1] = PID[1];
    this->_iPID[1] = PID[1];
    this->_tdPID[0] = 0;
    this->_tdPID[1] = 0;
    this->_tdPID[2] = 0;
    this->_level = level;
    for (int i = 0; i < this->_level; i++)
    {
        this->_err[i] = err[i];
        this->_derr[i] = derr[i];
        for (int j = 0; j < this->_level; j++)
        {
            this->_PRules[i][j] = PRules[i][j];
            this->_IRules[i][j] = IRules[i][j];
            this->_DRules[i][j] = DRules[i][j];
        }
    }
    for (int i = 0; i < this->_level; i++)
    {
        if (i == 0)
        {
            _Err[0].Init(-10000, -10000, this->_err[0], this->_err[1]);
            _DErr[0].Init(-100000, -100000, this->_derr[0], this->_derr[1]);
        }
        else if (i == this->_level - 1)
        {
            _Err[i].Init(this->_err[i - 1], this->_err[i], 10000, 10000);
            _DErr[i].Init(this->_derr[i - 1], this->_derr[i], 100000, 100000);
        }
        else
        {
            _Err[i].Init(this->_err[i - 1], this->_err[i], this->_err[i], this->_err[i + 1]);
            _DErr[i].Init(this->_derr[i - 1], this->_derr[i], this->_derr[i], this->_derr[i + 1]);
        }
    }
}

void FuzzyPID::_tunePID(float errVal, float derrVal)
{
    float sumP = 0;
    float sumI = 0;
    float sumD = 0;
    float sumM = 0;

    for (int i = 0; i < this->_level; i++)
    {
        for (int j = 0; j < this->_level; j++)
        {
            if (this->_Err[i].inrange(errVal) && _DErr[j].inrange(derrVal))
            {
                // Serial.print(i);
                // Serial.print(" ");
                // Serial.println(j);
                sumP += this->_PRules[j][i] * min(_Err[i].membership(errVal), _DErr[j].membership(derrVal));
                sumI += this->_IRules[j][i] * min(_Err[i].membership(errVal), _DErr[j].membership(derrVal));
                sumD += this->_DRules[j][i] * min(_Err[i].membership(errVal), _DErr[j].membership(derrVal));
                sumM += min(_Err[i].membership(errVal), _DErr[j].membership(derrVal));
            }
        }
    }

    this->_tdPID[0] = (sumM == 0) ? 0 : sumP / sumM;
    this->_tdPID[1] = (sumM == 0) ? 0 : sumI / sumM;
    this->_tdPID[2] = (sumM == 0) ? 0 : sumD / sumM;
    // this->_tdPID[0] = sumP / sumM;
    // this->_tdPID[1] = sumI / sumM;
    // this->_tdPID[2] = sumD / sumM;
}

float FuzzyPID::Result(float errVal, float derrVal, float ierrVal)
{
    this->_tunePID(errVal, derrVal);
    // Serial.print(errVal);
    // Serial.print(" ");
    // Serial.print(derrVal);
    // Serial.print(" ");
    // Serial.print(double((_iPID[0] + _tdPID[0]) * 180 / M_PI));
    // Serial.print(" ");
    // Serial.print(double((_iPID[1] + _tdPID[1]) * 180 / M_PI));
    // Serial.print(" ");
    // Serial.println(double((_iPID[2] + _tdPID[2]) * 180 / M_PI));
    float result = errVal * (_iPID[0] + _tdPID[0]) + ierrVal * (_iPID[1] + _tdPID[1]) + derrVal * ((_iPID[2] + _tdPID[2]));
    return result;
}