#include "carrosel.h"
#include "robot.h"

#define Vel_Lenta 0.1
#define Vel_Rapida 0.5

#include <Arduino.h>
#include <math.h>
#include "IRLine.h"

template <typename T> int sign(T val) 
{
  return (T(0) < val) - (val < T(0));
}

carrosel_t carrosel;

carrosel_t::carrosel_t(void)
{
 for (int i = 0; i < 4; i++)
 {
  idx_pos[i] = 90 * i;
 }
 p_e = 0;
 ve = 0;
 we = 0;
}

void carrosel_t::odometry(void)
{
 
 we = carrosel.enc * (TWO_PI / (64 * 2.0 * 1920.0 * 11));
 p_e += we*dt;
}

void carrosel_t::pos_update(void)
{
}

void carrosel_t::carrosel_pos_init(void)
{
  while (robot.IRLine.IR_values[0] > 60)
 {
  // Roda devagar se estiver perto da risca
  control_mode = carrosel_cm_voltage;
  u_req = 1;
 }
 // Corrigir odometria
 u_req = 0;
 control_mode = carrosel_cm_pos;
 p_e = 0;
}
/* bool carrosel_t::set_pos(float pos)
{
  while (carrosel.IR.IR_values[0] < 300)
 {
  // Roda devagar se estiver perto da risca
  if (abs(pos_e - pos) < 15)
  {
   carrosel.set_w(Vel_Lenta);
  }
  else
  {
   carrosel.set_w(Vel_Rapida);
  }
 }
 // Corrigir odometria
 carrosel.set_w(0);
 pos_e = pos;
 return true; 
 
}*/

void carrosel_t::set_w(float w)
{

 w_req=w;
}

void carrosel_t :: calcMotorsVoltage(void){

  if(control_mode == carrosel_cm_pos)
    {
        p_ref = p_req;
        PID.last_pe = PID.pos_error;
        PID.pos_error = p_ref-p_e;
        w1ref = PID.ppars->Kc_p*PID.pos_error + PID.ppars->Kd_p*(PID.pos_error - PID.last_pe);///PID.ppars->dt;
    
        if((abs(PID.pos_error)<0.005)&&(fabs(w1ref < 0.015))) u=0; //Cycle Oscillation Limiter
        else u = PID.calc(w1ref, we) + sign(w1ref) *  PID.ppars->dead_zone;
    }
    else if(control_mode == carrosel_cm_voltage)
    {
        u = u_req;
        return;
    }

    if (control_mode != carrosel_cm_pos) {
        if (w1ref != 0) u = PID.calc(w1ref, we) + sign(w1ref) *  PID.ppars->dead_zone;
        else {
          u = 0;
          PID.Se = 0;
          PID.y_ref = 0;
        }
    }
}



void carrosel_t::send_command(const char* command, float par)
{
  if (pchannels) pchannels->send_command(command, par);
}

void carrosel_t::send_command(const char* command, const char* par)
{
  if (pchannels) pchannels->send_command(command, par);
}