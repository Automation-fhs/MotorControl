class PID
{
public:
    PID(float PIDparams[3]);
    void setPIDparams(float PIDparams[3]);
    float Result(float posErr, float veloErr, float integralErr);

private:
    float _PID[3];
};