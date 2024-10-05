#include "../Fuzzy/Fuzzy.h"

class FuzzyPID
{
public:
    FuzzyPID(float PID[3], float err[7], float derr[7], int level, float PRules[7][7], float IRules[7][7], float DRules[7][7]);
    float Result(float err, float derr, float ierr);

private:
    void _tunePID(float err, float derr);
    FuzzySet _Err[7];
    FuzzySet _DErr[7];
    float _iPID[3];
    float _tdPID[3];
    float _err[7];
    float _derr[7];
    int _level;
    float _PRules[7][7];
    float _IRules[7][7];
    float _DRules[7][7];
};