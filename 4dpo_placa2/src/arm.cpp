#include "arm.h"

#include <Arduino.h>
#include <math.h>
#include "IRLine.h"
#include <math.h>
#include "trajectories.h"
#include "robot.h"

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

void pos_init(void)
{
    while(!digitalRead(switchPIN))
    {
        arm.control_mode = arm_cm_voltage;
        arm.u_req = 0.5; // o braço deverá mover-se de modo a ativar o interruptor
    }
    arm.u_req = 0;
    arm.control_mode == arm_cm_pos;
    arm.p_e = 0;

}

void arm_t::set_pos(float pos)
{
    p = pos;
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