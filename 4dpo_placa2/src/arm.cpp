#include "arm.h"

#include <Arduino.h>
#include <math.h>
#include "IRLine.h"
#include <math.h>

template <typename T> int sign(T val) 
{
  return (T(0) < val) - (val < T(0));
}

arm_t arm;

arm_t::arm_t(void)
{
    p_e = 0;
    ve = 0; 
    we = 0;

    dt = 0.04;


}

void arm_t::odometry(void)
{
 we = arm.enc * (TWO_PI / (64 * 2.0 * 1920.0 * 5));
 p_e += we*dt;
}

void arm_t::pos_update(void)
{

}

void pos_init(void)
{
    while(!digitalRead(switchPIN))
    {
        arm.control_mode = cm_voltage;
        arm.u_req = 0.5; // o braço deverá mover-se de modo a ativar o interruptor
    }
    arm.u_req = 0;
    arm.control_mode == cm_pos;
    arm.p_e = 0;

}

void arm_t::set_pos(float pos)
{
    p_req = pos;
}

void arm_t::calculate_motors_voltage(void)
{
    if(control_mode == cm_pos)
    {
        p_ref = p_req;
        PID[0].last_pe = PID[0].pos_error;
        PID[0].pos_error = p_ref-p_e;
        w1ref = PID[0].ppars->Kc_p*PID[0].pos_error + PID[0].ppars->Kd_p*(PID[0].pos_error - PID[0].last_pe)/PID[0].ppars->dt;
    
        if((abs(PID[0].pos_error)<0.005)&&(fabs(w1ref < 0.015))) u=0; //Cycle Oscillation Limiter
        else u = PID[0].calc(w1ref, we) + sign(w1ref) *  PID[0].ppars->dead_zone; 
    }
    else if(control_mode == cm_voltage)
    {
        u = u_req;
    }

    if (control_mode != cm_pos) {
        if (w1ref != 0) u = PID[0].calc(w1ref, we) + sign(w1ref) *  PID[0].ppars->dead_zone;
        else {
          u = 0;
          PID[0].Se = 0;
          PID[0].y_ref = 0;
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