
#include "pico4drive.h"
#include <Arduino.h>

 

#define ENCA 2
#define ENCB 3

int driver = 0;
/*float pwm = 0.2;*/
int a, b;
volatile int posi = 0;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

void setup()
{
    Serial.begin(115200);
    pico4drive_init();
    pinMode(ENCA, INPUT_PULLUP);
    pinMode(ENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
    
    /*Serial.println("target pos");*/
}

void loop()
{

    // Read the position
    /*int pos = 0; 
    noInterrupts(); // disable interrupts temporarily while reading
    pos = posi;
    interrupts(); // turn interrupts back on

    Serial.println(pos);*/
    /*pico4drive_set_motor_pwm(driver, pwm);*/

    /*a = digitalRead(ENCA);
    b = digitalRead(ENCB);

    Serial.print("EncA: ");
    Serial.print(a);
    Serial.print("  ");
    Serial.print("EncB: ");
    Serial.println(b);*/

    // set target position
    int target = 1200;
    //int target = 250*sin(prevT/1e6);

    // PID constants
    float kp = 0.01;
    float kd = 0;
    float ki = 0.01;
    

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position
    int pos = 0; 
    noInterrupts(); // disable interrupts temporarily while reading
    pos = posi;
    interrupts(); // turn interrupts back on

    // error
    int e = pos - target;

    // derivative
    float dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    float pwm = u;

    pwm = constrain(pwm, -1.0, 1.0); 
    if (abs(pwm) < 0.05) pwm = 0;
    static float rampPwm = 0;
    rampPwm += (pwm - rampPwm) * 0.1;

    pico4drive_set_motor_pwm(driver, rampPwm);

    // store previous error
    eprev = e;

    Serial.print(target);
    Serial.print(" ");
    Serial.print(pos);
    Serial.println();
}

