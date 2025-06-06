#include "carrosel.h"
#include "pico4drive.h"
#define Vel_Lenta 0.1
#define Vel_Rapida 0.5

#include <Arduino.h>
#include <math.h>
#include "IRLine.h"

#define MOTOR1A_PIN 11
#define MOTOR1B_PIN 10

template <typename T> int sign(T val) 
{
  return (T(0) < val) - (val < T(0));
}

void cpy_readIRSensors(IRLine_t &IRLine)
{
  byte c; // Read the five IR sensors using the AD converter
  for (c = 0; c < IRSENSORS_COUNT; c++)
  {
    IRLine.IR_values[(IRSENSORS_COUNT - 1) - c] = 1023 - pico4drive.read_adc(3 + c);
  }
}



carrosel_t carrosel;

carrosel_t::carrosel_t(void)
{
 idle_pos[0] = -10;
 idle_pos[1] = -106;
 idle_pos[2] = -192;
 idle_pos[3] = -285;

 pick1_pos[0] = -10;
 pick1_pos[1] = -106;
 pick1_pos[2] = -192;
 pick1_pos[3] = -285;

 store_pos[0] = -14;
 store_pos[1] = -110;
 store_pos[2] = -197;
 store_pos[3] = -290;

 pick2_pos[3] = -344;
 pick2_pos[2] = -255;
 pick2_pos[1] = -164;
 pick2_pos[0] = -71;

 pre_pos[3] = -302;
 pre_pos[2] = -209;
 pre_pos[1] = -122;
 pre_pos[0] = -29;

 drop_pos[0] = 7;
 drop_pos[1] = -86;
 drop_pos[2] = -173;
 drop_pos[3] = -276;
 counter = 0;
 p_e = 0;
 ve = 0;
 we = 0;

 pick_ok = 0;
 pick_done = 0;
 drop_ok = 0;
 drop_done = 0;
}

void carrosel_t::odometry(void)
{
 
 we = carrosel.enc * (TWO_PI / (64 * 2.0 * 1920.0 * 11));
 p_e += we*dt;
}

void carrosel_t::pos_update(void)
{
}

void carrosel_t::set_pos(float pos)
{
  p_req = (pos*TWO_PI)/360;
}

void carrosel_t::carrosel_pos_init(void)
{
  cpy_readIRSensors(IR);
  while (IR.IR_values[0] > 60)
 {
  // Roda devagar se estiver perto da risca
  cpy_readIRSensors(IR);
  control_mode = carrosel_cm_voltage;
  u_req = 2.5;
  calcMotorsVoltage();
  PWM = pico4drive.voltage_to_PWM(u);
  pico4drive.set_driver_PWM(PWM, MOTOR1A_PIN, MOTOR1B_PIN);
 }
 // Corrigir odometria
 u_req = 0;
 calcMotorsVoltage();
 PWM = pico4drive.voltage_to_PWM(u);
 pico4drive.set_driver_PWM(PWM, MOTOR1A_PIN, MOTOR1B_PIN);
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

        if (u > 2) u = 2;
        if (u < -2) u = -2;
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