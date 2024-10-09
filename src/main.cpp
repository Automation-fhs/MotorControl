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

#define TIMER1_INTERVAL_MS 10
#define TIMER1_FREQUENCY (float)(1000.0f / TIMER1_INTERVAL_MS)

#define Enc_A 21
#define Enc_B 20
#define Enc_Z 19

#define Motor_PWM 4
#define Motor_Dir 3
#define Home_Sensor 25
#define PkgSensor 25
#define Spare_Sensor 3
#define HomeSpeed 30
#define Home_Offset_Addr 0
#define Open_Offset_Addr 1
#define Motor_Voltage 12
#define Enc_PPP 1000
#define CallibHome 340
#define Control_Pin 23
#define Home 0
#define Open 30

#define Controller 3 // 1 for PID controller || 2 for Fuzzy controller || 3 for fPID controller
#define Unit 1       // 1 for degree || 2 for radian

// put function declarations here:
MotorControl myMotor = MotorControl(12, Enc_PPP, 1, float(TIMER1_INTERVAL_MS) / 1000, Controller);
RotaryEncoder myEncoder = RotaryEncoder();

int contrl_signl = 0;
float setpoint = 0;
bool isHome = false;
bool armed = false;
int prev_Home;
float prev_setpoint;
bool test = false;
uint32_t sTimer;
bool newsetpoint = false;
bool homeErr = false;

void drive(int signal)
{
  if (signal >= 0)
  {
    analogWrite(Motor_Dir, 255);
    analogWrite(Motor_PWM, signal);
  }
  else
  {
    analogWrite(Motor_Dir, 0);
    analogWrite(Motor_PWM, -signal);
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
  analogWrite(Motor_PWM, 0);
}

void enc()
{
  myEncoder.upd_Pulse(digitalRead(Enc_A), digitalRead(Enc_B));
  bool cur_Home = digitalRead(Home_Sensor);
  uint32_t motorPos = myEncoder.getCurPulse();
  if (prev_Home == 1 && !cur_Home && motorPos >= CallibHome - 100 && motorPos <= CallibHome + 100 && setpoint == Home)
  {
    myEncoder.setCurPulse(CallibHome);
    Serial.println("Reset Home");
  };
}

void motorControl()
{
  if (isHome)
  {
    if (setpoint == Home && newsetpoint == false)
    {
      newsetpoint = true;
    }

    if (myEncoder.getCurPulse() / (2 * Enc_PPP) >= Open + 3)
    {
      contrl_signl -= HomeSpeed;
    }
    if (myEncoder.getCurPulse() / (2 * Enc_PPP) <= Home - 3)
    {
      contrl_signl += HomeSpeed;
    }

    contrl_signl = myMotor.CtrlSignl(setpoint, myEncoder.getCurPulse(), 1);
    // Serial.print(myEncoder.getCurPulse());
    // Serial.print(" ");
    // Serial.println(contrl_signl);

    if (setpoint == Open && newsetpoint == true && abs(myEncoder.getCurPulse() / (2 * Enc_PPP) - Open) <= 1 && abs(contrl_signl) <= 10 && !digitalRead(Home_Sensor))
    {
      Serial.println("Fault Home Position");
      newsetpoint = false;
      drive(0);
      armed = false;
      while (!digitalRead(Home_Sensor))
      {
        drive(HomeSpeed);
      }
      drive(0);
      homeErr = true;
    }
    if (setpoint == Home && newsetpoint == true && abs(myEncoder.getCurPulse() / (2 * Enc_PPP) - Home) <= 1 && abs(contrl_signl) <= 0 && digitalRead(Home_Sensor))
    {
      Serial.println("Fault Home Position");
      newsetpoint = false;
      drive(0);
      armed = false;
      delay(5);
      homeMode();
      armed = true;
    }
  }

  if (armed)
  {
    drive(contrl_signl);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello World");
  pinMode(Enc_A, INPUT);
  pinMode(Enc_B, INPUT);
  pinMode(Motor_Dir, OUTPUT);
  pinMode(Motor_PWM, OUTPUT);

  //------------------- Control Init ---------------------
  pinMode(PkgSensor, INPUT);
  pinMode(Control_Pin, INPUT);

  myEncoder.encInit(digitalRead(Enc_A), digitalRead(Enc_B));
  attachInterrupt(digitalPinToInterrupt(Enc_A), enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_B), enc, CHANGE);

  ITimer1.init();
  if (ITimer1.attachInterrupt(TIMER1_FREQUENCY, motorControl))
    Serial.println("Starting Timer1 OK!");
  else
    Serial.println("Can't set Timer1");
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
    else
    {
      setpoint = Home;
      test = false;
    }
  }
}

// put function definitions here: