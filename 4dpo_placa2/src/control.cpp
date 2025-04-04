#include <Arduino.h>
#include "carrosel.h"
#include "arm.h"
#include "state_machines.h"
#include "motor_bench.h"




motor_bench_t motor_bench;

class Roda_t : public state_machine_t
{
  virtual void next_state_rules(void)
  {
    // idle
    if (state == 0 && tis > 4 && carrosel.pick_ok == true)
    {
      set_new_state(1);
      carrosel.pick_done = false;
      carrosel.pick_ok = false;
    }
    // pick
    else if (state == 1 && tis > 3.5)
    {
      set_new_state(2);
    }
    else if (state == 2 && tis > 1)
    {
      set_new_state(3);
    }
    // store
    else if (state == 3 && tis > 4.5)
    {
      set_new_state(4);
    }
    else if (state == 4 && tis > 4.5)
    {
      set_new_state(5);
    }
    else if (state == 5 && tis > 1  && carrosel.counter < 3 )
    {
      carrosel.counter++;
      carrosel.pick_done = true;
      set_new_state(0);
    }
    // pick from carrosel
    else if (state == 5  && tis > 3)
    {
      set_new_state(6);
    }
    else if (state == 6 && tis > 1 && carrosel.drop_ok == true)
    {
      set_new_state(7);
      carrosel.drop_ok = false;
      carrosel.drop_done = false;
    }
    else if (state == 7 && tis > 5)
    {
      set_new_state(8);
    }
    else if (state == 8 && tis > 2)
    {
      set_new_state(9);
    }
    else if (state == 9 && tis > 4)
    {
      set_new_state(10);
    }
    //drop
    else if (state == 10 && tis > 1)
    {
      set_new_state(11);
    }
    else if (state == 11 && tis > 4.5)
    {
      set_new_state(12);
    }
    else if (state == 12 && tis > 4.5)
    {
      set_new_state(6);
    }
    else if (state == 12 && tis > 1 && carrosel.counter  > 0)
    {
      carrosel.counter--;
      carrosel.drop_done = true;
      set_new_state(6);
    }
    else if (state == 12 && tis > 1)
    {
      set_new_state(13);
    }
    else if (state == 13 && tis > 5 )
    {
      set_new_state(13);
      carrosel.counter = 0;
    }
    //end
  };

  virtual void state_actions_rules(void)
  {
    // Actions in each state
    if (state == 0)
    { 
      arm.set_pos(-45);
      carrosel.set_pos(carrosel.idle_pos[carrosel.counter] +10);
    }
    else if (state == 1)
    { 
      arm.set_pos(-2);
      carrosel.set_pos(carrosel.pick1_pos[carrosel.counter]) ;
    }
    else if (state == 2)
    { 
      carrosel.solenoid_PWM = 1000;
    }
    else if (state == 3)
    { 
      arm.set_pos(-67);
      carrosel.set_pos(carrosel.store_pos[carrosel.counter]);
      carrosel.solenoid_PWM = 1000;
    }
    else if (state == 4)
    { 
      carrosel.set_pos(carrosel.store_pos[carrosel.counter] );
    }
    else if (state == 5)
    { 
      carrosel.solenoid_PWM = 0;
    }
    else if (state == 6)
    { 
      carrosel.set_pos(carrosel.pick2_pos[carrosel.counter]+ 40);
    }
    else if (state == 7)
    { 
      arm.set_pos(-80);
    }
    else if (state == 8)
    { 
      carrosel.set_pos(carrosel.pick2_pos[carrosel.counter]);
    }
    else if (state == 9)
    {
      carrosel.solenoid_PWM = 1000;
    }
    else if (state == 10)
    {
      arm.set_pos(-80); // -81
      carrosel.set_pos(carrosel.pre_pos[carrosel.counter]);
      carrosel.solenoid_PWM = 1000;
    }
    else if (state == 11)
    {
      arm.set_pos(-5);
      carrosel.set_pos(carrosel.drop_pos[carrosel.counter]);
      carrosel.solenoid_PWM = 1000;
    }
    else if (state == 12)
    {
      carrosel.solenoid_PWM = 0;
    }
    else if (state == 13)
    {
      arm.set_pos(-45);
      carrosel.set_pos(-10);
    }
    else if (state == 300)
    {
      carrosel.control_mode = carrosel_cm_pos;
    }
  };
};

Roda_t roda;

void init_control(arm_t &arm, carrosel_t &carrosel)
{
  carrosel.pfsm = &roda;
  arm.pfsm = &roda;
  roda.set_new_state(0);
  roda.update_state();
  state_machines.register_state_machine(&roda);
}

void control(arm_t &arm, carrosel_t &carrosel)
{
  arm.control_mode = arm_cm_pos;
  carrosel.control_mode = carrosel_cm_pos;
  //roda.set_new_state(1);
  //roda.update_state();
  
  state_machines.step();
}