#include "carrosel.h"

#define Vel_Lenta 0.1
#define Vel_Rapida 0.5

#include <Arduino.h>
#include <math.h>
#include "IRLine.h"

carrosel_t carrosel;

carrosel_t::carrosel_t(void)
{
 for (int i = 0; i < 4; i++)
 {
  pos[i] = 90 * i;
 }
 pos_e = 0;
 ve = 0;
 we = 0;
}

void carrosel_t::odometry(void)
{
 // Falta arranjar a constante adequada para a roda grande
 we = carrosel.enc1 * (TWO_PI / (64 * 2.0 * 1920.0));
}

void carrosel_t::pos_update(void)
{
}

bool carrosel_t::set_pos(float pos)
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
}

void carrosel_t::set_w(float w)
{

 w_req=w;
}

void carrosel_t::calculate_motors_voltage(void)
{
 if (w_req != 0)
  u1 = PID[0].calc(w_req, we) - w_req * PID[0].ppars->dead_zone;
}