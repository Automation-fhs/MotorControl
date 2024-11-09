#define TIMER_INTERRUPT_DEBUG 2
#define _TIMERINTERRUPT_LOGLEVEL_ 0

#define USE_TIMER_1 true

#if (defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) ||                   \
     defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_ETHERNET) ||                        \
     defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT) || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO) ||                            \
     defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) ||    \
     defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
     defined(ARDUINO_AVR_PROTRINKET3FTDI))
#define USE_TIMER_2 true
#else
#define USE_TIMER_3 true
#endif

#include <Arduino.h>
#include "MotorControl.h"
#include <TimerInterrupt.h>
#include "./Encoder/RotaryEncoder.h"

// ---- Timer ----
#define TIMER1_INTERVAL_MS 10
#define TIMER1_FREQUENCY (float)(1000.0f / TIMER1_INTERVAL_MS)

// ---- Encoder ----
#define Enc_A 21
#define Enc_B 20
#define Enc_Z 19

// ---- Sensor ----
#define Home_Sensor 18
#define PkgSensor 25
#define Spare_Sensor

// ---- Control ----
#define Motor_PWM 5
#define Motor_Dir 4
#define Control_Pin 23 // Receiving Open/Close signal from old center controller

// ---- Button ----
#define Open_BTN 42
#define Close_BTN 44
#define Spare_BTN 40

// ---- Lagging Prevent Pin ----
#define NL_Pin 2 // Change pwm continuously to make sure Arduino is running properly

// ---- Arm parameters ----
#define HomeSpeed 30
#define Motor_Voltage 12
#define Enc_PPP 1000
#define CallibHome 300
#define Home 0
#define Open 30

// ---- Controller parameters ----
#define Controller 3 // 1 for PID controller || 2 for Fuzzy controller || 3 for fPID controller
#define Unit 1       // 1 for degree || 2 for radian

// put function declarations here:
MotorControl myMotor = MotorControl(12, Enc_PPP, 1, float(TIMER1_INTERVAL_MS) / 1000, Controller); // Voltage - pulse per phase - gear ratio - time interval - controller
RotaryEncoder myEncoder = RotaryEncoder();

int contrl_signl = 0; // PWM signal sent to driver
float setpoint = 0;   // desired arm's position
bool isHome = false;  // find home once when initialized
bool armed = false;   // run controller (false for turn off controller)
int prev_Home;
float prev_setpoint;
bool test = false;
uint32_t sTimer;
bool newsetpoint = false;
bool homeErr = false;
int NL_Sgnl;
int cur_P = 0;
bool errState = false;
bool test = false;
uint32_t homeNR;
bool homeNR_Flag;
uint32_t control_timer;
int prev_control;
long prev_pos;
bool home_err = false;
bool enc_err_test;
bool ard_err_test;

void drive(int signal)
{
  if (signal >= 0)
  {
    analogWrite(Motor_PWM, signal);
    analogWrite(Motor_Dir, 255);
  }
  else
  {
    analogWrite(Motor_PWM, -signal);
    analogWrite(Motor_Dir, 0);
  }
}

void homeMode()
{
  Serial.println("Finding Home");
  if (digitalRead(Home_Sensor))
  {
    drive(-HomeSpeed);
    while (digitalRead(Home_Sensor))
    {
      Serial.println("Finding Home");
      drive(-HomeSpeed);
    }
    myEncoder.setCurPulse(CallibHome);
    Serial.println("Home found!");
  }
  else
  {
    drive(HomeSpeed);
    while (!digitalRead(Home_Sensor))
    {
      Serial.println("Finding Home");
      drive(HomeSpeed);
    }
    drive(0);
    while (digitalRead(Home_Sensor))
    {
      drive(-HomeSpeed);
      ;
    }
    myEncoder.setCurPulse(CallibHome);
    Serial.println("Home found!");
  }
  drive(0);
}

void enc()
{
  if (!enc_err_test)
    myEncoder.upd_Pulse(digitalRead(Enc_A), digitalRead(Enc_B));
}

void motorControl()
{
  if (!errState && !ard_err_test)
  {
    analogWrite(NL_Pin, NL_Sgnl);
    if (NL_Sgnl >= 250)
      NL_Sgnl = 0;
    else
      NL_Sgnl += 5;
  }
  if (setpoint == Home && newsetpoint == false)
    newsetpoint = true;

  if (armed && !errState)
  {
    contrl_signl = myMotor.CtrlSignl(setpoint, myEncoder.getCurPulse(), 1);

    float curDeg = float(myEncoder.getCurPulse()) / (2 * Enc_PPP);

    if (curDeg >= Open + 3)
    {
      contrl_signl -= HomeSpeed;
    }
    if (curDeg <= Home - 3)
    {
      contrl_signl += HomeSpeed;
    }

    if (setpoint == Open && newsetpoint && abs(curDeg - Open) <= 2.5 && contrl_signl <= HomeSpeed && !digitalRead(Home_Sensor))
    {
      Serial.println("Fault Home Position");
      newsetpoint = false;
      drive(0);
      contrl_signl = 0;
      armed = false;
      home_err = true;
    }

    if (abs(contrl_signl) <= HomeSpeed + 10)
    {
      control_timer = millis();
    }

    if (millis() - control_timer >= 200 && abs(myEncoder.getCurPulse() - prev_pos) <= 1)
    {
      drive(0);
      errState = true;
      armed = false;
      contrl_signl = 0;
      Serial.println("Encoder Error!!");
    }

    drive(contrl_signl);
    prev_pos = myEncoder.getCurPulse();
  }

  if (home_err)
  {
    if (!digitalRead(Home_Sensor))
    {
      drive(HomeSpeed);
    }
    else
    {
      armed = true;
      home_err = false;
    }
  }
}

void home()
{
  if (!digitalRead(Home_Sensor))
  {
    homeNR = millis();
    homeNR_Flag = true;
  }
  else
    homeNR_Flag = false;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello World");
  pinMode(Enc_A, INPUT);
  pinMode(Enc_B, INPUT);

  pinMode(Home_Sensor, INPUT);
  pinMode(Motor_Dir, OUTPUT);
  pinMode(Motor_PWM, OUTPUT);

  //------------------- Control Init ---------------------
  pinMode(PkgSensor, INPUT);
  pinMode(Control_Pin, INPUT);
  pinMode(NL_Pin, OUTPUT);

  myEncoder.encInit(digitalRead(Enc_A), digitalRead(Enc_B));
  attachInterrupt(digitalPinToInterrupt(Enc_A), enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_B), enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Home_Sensor), home, CHANGE);

  ITimer1.init();
  if (ITimer1.attachInterrupt(TIMER1_FREQUENCY, motorControl))
    Serial.println("Starting Timer1 OK!");
  else
    Serial.println("Can't set Timer1");

  errState = false;
  enc_err_test = false;
  ard_err_test = false;
  NL_Sgnl = 0;
}

void loop()
{
  if (!isHome)
  {
    homeMode();
    Serial.println("Home found!");
    isHome = true;
    armed = true;
    setpoint = Home;
    prev_Home = digitalRead(Home_Sensor);
    prev_setpoint = setpoint;
  }

  if (homeNR_Flag && millis() - homeNR >= 50)
  {
    if (myEncoder.getCurPulse() >= CallibHome - 100 && myEncoder.getCurPulse() <= CallibHome + 100)
    {
      Serial.print("Callib current pulse: ");
      Serial.println(myEncoder.getCurPulse());
      myEncoder.setCurPulse(CallibHome);
    }
    homeNR_Flag = false;
  }

  if (test == false)
  {
    if (!digitalRead(Control_Pin))
    {
      sTimer = millis();
      setpoint = Home;
      homeErr = false;
      armed = true;
    }
    if ((millis() - sTimer >= 100) && !homeErr)
    {
      setpoint = Open;
      // Serial.println("Arm Open");
    }
  }

  if (Serial.available())
  {
    int signal = Serial.read();
    if (signal == 49)
    {
      setpoint = Open;
      test = true;
    }
    else if (signal == 53)
      enc_err_test = true;
    else if (signal == 57)
      ard_err_test = true;
    else
    {
      setpoint = Home;
      test = false;
    }
  }
}

// put function definitions here: