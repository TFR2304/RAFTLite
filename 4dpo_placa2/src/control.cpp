#include <Arduino.h>
#include "carrosel.h"
#include "arm.h"
#include "state_machines.h"
#include "motor_bench.h"
#include "trajectories.h"

motor_bench_t motor_bench;

class Roda_t : public state_machine_t
{
  virtual void next_state_rules(void)
  {
    // Parado - apenas sai do estado se o mudarmos manualmente
    if (state == 0 && tis > 10)
    {
      set_new_state(1);
    }
    // Set_Pos
    else if (state == 1 && robot.tof_dist < 0.04)
    {
      set_new_state(2);
    }
    // Set_W
    else if (state == 2 && tis > 0.1)
    {
      robot.rel_s = 0;
      set_new_state(3);
    }
    else if (state == 3 && robot.rel_s < -0.12)
    {
      robot.rel_theta = 0;
      set_new_state(4);
    }
    else if (state == 4 && robot.rel_theta > radians(170))
    {
      set_new_state(0);
      robot.IRLine.crosses = 0;
    }
  };

  virtual void state_actions_rules(void)
  {
    // Actions in each state
    if (state == 0)
    { // Carrosel Parado
      //carrosel.carrosel_pos_init();
    }
    else if (state == 1)
    { // Go: Get first box
      //bool next = carrosel.set_pos((carrosel.pos_req-1) * 90);
    }
    else if (state == 2)
    { // Turn Solenoid On and Get the Box
      robot.solenoid_PWM = 180;
      robot.followLineLeft(robot.follow_v, robot.follow_k);
    }
    else if (state == 3)
    { // Go back with the box
      robot.solenoid_PWM = 180;
      robot.setRobotVW(-0.1, 0, 0);
    }
    else if (state == 4)
    { // Turn arround
      robot.solenoid_PWM = 180;
      robot.setRobotVW(0, 0, 2.5);
    }
    else if (state == 201)
    {
      carrosel.control_mode = carrosel_cm_voltage;
    }
    else if (state == 300)
    {
      carrosel.control_mode = carrosel_cm_pos;
    }
  };
};

Roda_t roda;

class Braço_t : public state_machine_t
{
  virtual void next_state_rules(void)
  {
    // Rules for the state evolution
    if (state == 0 && tis > 0.1)
    {
      set_new_state(1);
    }
    else if (state == 1 && tis > 0.2)
    {
      set_new_state(2);
    }
    else if (state == 2 && tis > 0.2)
    {
      set_new_state(3);
    }
    else if (state == 3 && tis > 0.6)
    {
      set_new_state(0);
    }
  };

  virtual void state_actions_rules(void)
  {
    // Actions in each state
    if (state == 0)
    { // LED on
      robot.led = 1;
    }
    else if (state == 1)
    { // LED off
      robot.led = 0;
    }
    else if (state == 2)
    { // LED on
      robot.led = 1;
    }
    else if (state == 3)
    { // LED off
      robot.led = 0;
    }
    /* else if (state == 4)
    {
      //arm.pos_init();
    } */
    else if (state == 201)
    {
      arm.control_mode = arm_cm_voltage;
    }
    else if (state == 300)
    {
      arm.control_mode = arm_cm_pos;
    }
  };
};

Braço_t braço;

void init_control(arm_t &arm, carrosel_t &carrosel)
{
  arm.pfsm = &braço;
  carrosel.pfsm = &roda;
  braço.set_new_state(300);
  braço.update_state();
  roda.set_new_state(300);
  roda.update_state();
  state_machines.register_state_machine(&braço);
  state_machines.register_state_machine(&roda);
}

void control(arm_t &arm, carrosel_t &carrosel)
{
  arm.control_mode = arm_cm_pos;
  carrosel.control_mode = carrosel_cm_pos;

  state_machines.step();
}

/*void init_control(carrosel_t &carrosel)
{
  carrosel.pfsm = &roda;
  roda.set_new_state(300);
  roda.update_state();
  state_machines.register_state_machine(&roda);
}

void control(carrosel_t &carrosel)
{
  carrosel.control_mode = carrosel_cmc_pos;

  state_machines.step();
}*/