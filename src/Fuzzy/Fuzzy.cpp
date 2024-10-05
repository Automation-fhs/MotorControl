#include "./Fuzzy.h"
#include <Arduino.h>

Fuzzy::Fuzzy(float errCP[7], float derrCP[7], int level, float FuzzyRules[7][7])
{
    this->_level = level;
    for (int i = 0; i < this->_level; i++)
    {
        this->_errCP[i] = errCP[i];
        this->_derrCP[i] = derrCP[i];
        for (int j = 0; j < this->_level; j++)
        {
            this->_FuzzyRules[i][j] = FuzzyRules[i][j];
        }
    }
    for (int i = 0; i < this->_level; i++)
    {
        if (i == 0)
        {
            // _Err[0].Init(this->_errCP[0], this->_errCP[0], this->_errCP[0], this->_errCP[1]);
            // _DErr[0].Init(this->_derrCP[0], this->_derrCP[0], this->_derrCP[0], this->_derrCP[1]);
            _Err[0].Init(-10000, -10000, this->_errCP[0], this->_errCP[1]);
            _DErr[0].Init(-100000, -100000, this->_derrCP[0], this->_derrCP[1]);
        }
        else if (i == this->_level - 1)
        {
            _Err[i].Init(this->_errCP[i - 1], this->_errCP[i], 10000, 10000);
            _DErr[i].Init(this->_derrCP[i - 1], this->_derrCP[i], 100000, 100000);
        }
        else
        {
            _Err[i].Init(this->_errCP[i - 1], this->_errCP[i], this->_errCP[i], this->_errCP[i + 1]);
            _DErr[i].Init(this->_derrCP[i - 1], this->_derrCP[i], this->_derrCP[i], this->_derrCP[i + 1]);
        }
    }
}

void Fuzzy::setParameter(float errCP[7], float derrCP[7], float FuzzyRules[7][7])
{
    for (int i = 0; i < this->_level; i++)
    {
        this->_errCP[i] = errCP[i];
        this->_derrCP[i] = derrCP[i];
        for (int j = 0; j < this->_level; j++)
        {
            this->_FuzzyRules[i][j] = FuzzyRules[i][j];
        }
    }
    for (int i = 0; i < this->_level; i++)
    {
        if (i == 0)
        {
            // _Err[0].Init(this->_errCP[0], this->_errCP[0], this->_errCP[0], this->_errCP[1]);
            // _DErr[0].Init(this->_derrCP[0], this->_derrCP[0], this->_derrCP[0], this->_derrCP[1]);
            _Err[0].Init(-10000, -10000, this->_errCP[0], this->_errCP[1]);
            _DErr[0].Init(-100000, -100000, this->_derrCP[0], this->_derrCP[1]);
        }
        else if (i == this->_level - 1)
        {
            _Err[i].Init(this->_errCP[i - 1], this->_errCP[i], 10000, 10000);
            _DErr[i].Init(this->_derrCP[i - 1], this->_derrCP[i], 100000, 100000);
        }
        else
        {
            _Err[i].Init(this->_errCP[i - 1], this->_errCP[i], this->_errCP[i], this->_errCP[i + 1]);
            _DErr[i].Init(this->_derrCP[i - 1], this->_derrCP[i], this->_derrCP[i], this->_derrCP[i + 1]);
        }
    }
}

float Fuzzy::Result(float errVal, float derrVal)
{
    float sumVal = 0;
    float sumMem = 0;

    for (int i = 0; i < this->_level; i++)
    {
        for (int j = 0; j < this->_level; j++)
        {
            if (_Err[i].inrange(errVal) && _DErr[j].inrange(derrVal))
            {
                // Serial.print(i);
                // Serial.print(" ");
                // Serial.println(j);
                sumVal += this->_FuzzyRules[j][i] * min(_Err[i].membership(errVal), _DErr[j].membership(derrVal));
                // console.log(`Value at pos ${i} vel ${j} is ${Math.min(Position[i].membership(errVal),Velo[j].membership(derrVal))}`);
                sumMem += min(_Err[i].membership(errVal), _DErr[j].membership(derrVal));
            }
        }
    }
    // Serial.print(double(sumVal));
    // Serial.print(" ");
    // Serial.print(double(sumMem));
    // Serial.print(" ");
    if (sumMem == 0)
        return 0;
    else
        return sumVal / sumMem;
}