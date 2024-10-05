#include "./PID.h"

PID::PID(float PIDparams[3])
{
    this->_PID[0] = PIDparams[0];
    this->_PID[1] = PIDparams[1];
    this->_PID[2] = PIDparams[2];
}

void PID::setPIDparams(float PIDparams[3])
{
    this->_PID[0] = PIDparams[0];
    this->_PID[1] = PIDparams[1];
    this->_PID[2] = PIDparams[2];
}

float PID::Result(float posErr, float veloErr, float integralErr)
{
    return posErr * this->_PID[0] + integralErr * this->_PID[1] + veloErr * this->_PID[2];
}