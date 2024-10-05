#include "./FuzzySet.h"

class Fuzzy
{
public:
    Fuzzy(float err[7], float derr[7], int level, float FuzzyRules[7][7]);
    void setParameter(float err[7], float derr[7], float FuzzyRules[7][7]);
    float Result(float posVal, float veloVal);

private:
    FuzzySet _Err[7];
    FuzzySet _DErr[7];
    int _level;
    float _errCP[7];
    float _derrCP[7];
    float _FuzzyRules[7][7];
};