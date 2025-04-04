#include "arm.h"
#include "pico4drive.h"
#include <Arduino.h>
#include <math.h>
#include "IRLine.h"
#include <math.h>

#define MOTOR2A_PIN 13
#define MOTOR2B_PIN 12



template <typename T> int sign(T val) 
{
  return (T(0) < val) - (val < T(0));
}

arm_t arm;


arm_t::arm_t()
{
    p_e = 0;
    ve = 0; 
    we = 0;

    dt = 0.04;

    pfsm = NULL;
    pchannels = NULL;
}

void arm_t::odometry(void)
{
 we = arm.enc * (TWO_PI / (64 * 2.0 * 1920.0 * 5));
 p_e += we*dt;
}

void arm_t::pos_update(void)
{

}

void arm_t::pos_init(void)
{

  while(digitalRead(switchPIN) == 1)
  {
      control_mode = arm_cm_voltage;
      u_req = 1.5;
      calcMotorsVoltage();
      PWM = pico4drive.voltage_to_PWM(u);
      pico4drive.set_driver_PWM(PWM, MOTOR2A_PIN, MOTOR2B_PIN); // o braço deverá mover-se de modo a ativar o interruptor
  }
  u_req = 0;
  p_e = 0;
  calcMotorsVoltage();
  PWM = pico4drive.voltage_to_PWM(u);
  pico4drive.set_driver_PWM(PWM, MOTOR2A_PIN, MOTOR2B_PIN);
  control_mode = arm_cm_pos;
  set_pos(-45);
}

void arm_t::set_pos(float pos)
{
  p_req = (pos*TWO_PI)/360;
}

void arm_t::calcMotorsVoltage(void)
{
    if(control_mode == arm_cm_pos)
    {
        p_ref = p_req;
        PID.last_pe = PID.pos_error;
        PID.pos_error = p_ref-p_e;
        w1ref = PID.ppars->Kc_p*PID.pos_error + PID.ppars->Kd_p*(PID.pos_error - PID.last_pe);///PID.ppars->dt;
    
        if((abs(PID.pos_error)<0.005)&&(fabs(w1ref < 0.015))) u=0; //Cycle Oscillation Limiter
        else u = PID.calc(w1ref, we) + sign(w1ref) *  PID.ppars->dead_zone;
        if (u > 2) u = 1.2;
        if (u < -2) u = -1.2;
    }
    else if(control_mode == arm_cm_voltage)
    {
        u = u_req;
        return;
    }

    if (control_mode != arm_cm_pos) {
        if (w1ref != 0) u = PID.calc(w1ref, we) + sign(w1ref) *  PID.ppars->dead_zone;
        else {
          u = 0;
          PID.Se = 0;
          PID.y_ref = 0;
        }
    }

}

void arm_t::send_command(const char* command, float par)
{
  if (pchannels) pchannels->send_command(command, par);
}

void arm_t::send_command(const char* command, const char* par)
{
  if (pchannels) pchannels->send_command(command, par);
}