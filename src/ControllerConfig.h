#include <math.h>

// =======================================================================================

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Fuzzy Init ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// =======================================================================================
// ----- Error in degree -----
#define LE 10 //<--Edit--
#define ME 25 //<--Edit--
#define HE 45 //<--Edit--
float err[7] = {-HE, -ME, -LE, 0, LE, ME, HE};

// ---- Change of Error in degree/s ----
#define LCE 200  //<--Edit--
#define MCE 600  //<--Edit--
#define HCE 5000 //<--Edit--
float derr[7] = {-HCE, -MCE, -LCE, 0, LCE, MCE, HCE};

// --------- Fuzzy Rules ---------
#define ZO 0
#define P1 125
#define P2 200
#define P3 255
#define P4 255
#define P5 255
#define P6 255
#define N1 -P1
#define N2 -P2
#define N3 -P3
#define N4 -P4
#define N5 -P5
#define N6 -P6

float FuzzyRules[7][7] = {
    {P6, P5, P4, P3, P2, P1, ZO},
    {P5, P4, P3, P2, P1, ZO, N1},
    {P4, P3, P2, P1, ZO, N1, N2},
    {P3, P2, P1, ZO, N1, N2, N3},
    {P2, P1, ZO, N1, N2, N3, N4},
    {P1, ZO, N1, N2, N3, N4, N5},
    {ZO, N1, N2, N3, N4, N5, N6}};
// =======================================================================================

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PID Init ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// =======================================================================================

float PIDdegree[3] = {10, 1, 4};
float PID_Scale_Factor = 1;
float PIDfactor[3] = {PID_Scale_Factor*PIDdegree[0] * M_PI / 180, PID_Scale_Factor*PIDdegree[1] * M_PI / 180, PID_Scale_Factor*PIDdegree[2] * M_PI / 180};

// =======================================================================================

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ FuzzyPID Init ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// =======================================================================================
#define LFE 10
#define MFE 25
#define HFE 45
float FErr[7] = {-HFE, -MFE, -LFE, 0, LFE, MFE, HFE};

#define LCFE 100
#define MCFE 500
#define HCFE 1000
float dFErr[7] = {-HCFE, -MCFE, -LCFE, 0, LCFE, MCFE, HCFE};

// --------- P Rules ---------
#define PZO 0
#define PP1 PIDfactor[0] / 6
#define PP2 PIDfactor[0] / 3
#define PP3 PIDfactor[0] / 2
#define PP4 2 * PIDfactor[0] / 3
#define PP5 5 * PIDfactor[0] / 6
#define PP6 PIDfactor[0]
#define PN1 -PP1
#define PN2 -PP2
#define PN3 -PP3
#define PN4 -PP4
#define PN5 -PP5
#define PN6 -PP6

float PRules[7][7] = {
    {PP6, PP6, PP6, PZO, PZO, PZO, PP6},
    {PP6, PP6, PP4, PZO, PZO, PP2, PP6},
    {PP6, PP6, PP2, PZO, PZO, PP4, PP6},
    {PP6, PP6, PZO, PZO, PZO, PP6, PP6},
    {PP6, PP4, PZO, PZO, PP2, PP6, PP6},
    {PP6, PP2, PZO, PZO, PP4, PP6, PP6},
    {PP6, PZO, PZO, PZO, PP6, PP6, PP6}};

// --------- I Rules ---------
#define IZO 0
#define IP1 PIDfactor[1] / 6
#define IP2 PIDfactor[1] / 3
#define IP3 PIDfactor[1] / 2
#define IP4 PIDfactor[1] * 2 / 3
#define IP5 PIDfactor[1] * 5 / 6
#define IP6 PIDfactor[1]
#define IN1 -IP1
#define IN2 -IP2
#define IN3 -IP3
#define IN4 -IP4
#define IN5 -IP5
#define IN6 -IP6

float IRules[7][7] = {
    {IP6, IP6, IP6, IN4, IN4, IN2, IP2},
    {IP6, IP6, IP4, IN4, IN4, IZO, IP4},
    {IP6, IP6, IP2, IN4, IN2, IP2, IP6},
    {IP6, IP4, IZO, IN4, IZO, IP4, IP6},
    {IP6, IP2, IN2, IN4, IP2, IP6, IP6},
    {IP4, IZO, IN4, IN4, IP4, IP6, IP6},
    {IP2, IN2, IN4, IN4, IP6, IP6, IP6}};

// --------- D Rules ---------
#define DZO 0
#define DP1 PIDfactor[2] / 6
#define DP2 PIDfactor[2] / 3
#define DP3 PIDfactor[2] / 2
#define DP4 PIDfactor[2] * 2 / 3
#define DP5 PIDfactor[2] * 5 / 6
#define DP6 PIDfactor[2]
#define DN1 -DP1
#define DN2 -DP2
#define DN3 -DP3
#define DN4 -DP4
#define DN5 -DP5
#define DN6 -DP6

// float DRules[7][7] = {
//     {DN6, DN6, DN2, DP6, DP6, DP6, DN6},
//     {DN6, DN6, DZO, DP6, DP6, DP4, DN6},
//     {DN6, DN3, DP2, DP6, DP6, DP2, DN6},
//     {DN6, DZO, DP4, DP6, DP4, DZO, DN6},
//     {DN6, DP2, DP6, DP6, DP2, DN3, DN6},
//     {DN6, DP4, DP6, DP6, DZO, DN6, DN6},
//     {DN6, DP6, DP6, DP6, DN2, DN6, DN6}};

float DRules[7][7] = {
    {DN6, DN6, DP2, DZO, DP6, DP6, DN6},
    {DN6, DN6, DP4, DZO, DP6, DP4, DN6},
    {DN6, DN3, DP6, DZO, DP6, DP2, DN6},
    {DN6, DZO, DP6, DZO, DP6, DZO, DN6},
    {DN6, DP2, DP6, DZO, DP6, DN3, DN6},
    {DN6, DP4, DP6, DZO, DP4, DN6, DN6},
    {DN6, DP6, DP6, DZO, DP2, DN6, DN6}};