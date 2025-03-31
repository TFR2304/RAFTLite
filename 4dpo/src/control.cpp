#include <Arduino.h>
#include "robot.h"
#include "state_machines.h"
#include "motor_bench.h"
#include "trajectories.h"

motor_bench_t motor_bench;

class main_fsm_t : public state_machine_t
{
  virtual void next_state_rules(void)
  {
    // Rules for the state evolution
    if (state == 0 && robot.tof_dist > 0.10 && robot.prev_tof_dist < 0.05 && tis > 1.0)
    {
      robot.rel_s = 0;
      set_new_state(1);
    }
    else if (state == 1 && robot.TouchSwitch)
    {
      robot.at_destin = false;
      set_new_state(2);
    }
    else if (state == 2 && robot.at_destin && 0)
    {
      robot.at_destin = false;
      set_new_state(3);
    }
    else if (state == 3)
    {
      // robot.rel_theta = 0;
      // set_new_state(4);
    }
    else if (state == 4 && robot.rel_theta > radians(170))
    {
      set_new_state(5);
      // robot.IRLine.crosses = 0;
    }
    else if (state == 5 /*&& Rrobot.IRLine.crosses >= 5*/)
    {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      set_new_state(6);
    }
    else if (state == 6 && robot.rel_theta < radians(-70) /*&& robot.IRLine.total > 1500*/)
    {
      set_new_state(7);
    }
    else if (state == 7 && tis > 2.0)
    {
      // robot.IRLine.crosses = 0;
      set_new_state(8);
    }
    else if (state == 202 && tis > 2.0)
    {
      // robot.IRLine.crosses = 0;
      set_new_state(200);
    }
    else if (state == 203 && actions_count > 0)
    {
      set_new_state(prev_state);
    }
    else if (state == mb_heat_motor && tis > motor_bench.warm_up_time)
    {
      robot.u1_req = 0;
      set_new_state(mb_goto_voltage);
    }
    else if (state == mb_goto_voltage && tis > motor_bench.settle_time)
    {
      motor_bench.acc_i = 0;
      motor_bench.acc_w = 0;
      motor_bench.n = 0;
      set_new_state(mb_acc_measure);
    }
    else if (state == mb_acc_measure && motor_bench.n > motor_bench.total_measures)
    {
      int i;
      char scratch[256];

      snprintf(scratch, 256, "%.6g %.6g %.6g", robot.u1, motor_bench.acc_i / motor_bench.n, motor_bench.acc_w / motor_bench.n);
      robot.pchannels->send_string(scratch, true);

      robot.u1_req += motor_bench.voltage_step;
      if (robot.u1_req <= motor_bench.max_voltage)
      {
        motor_bench.acc_i = 0;
        motor_bench.acc_w = 0;
        motor_bench.n = 0;
        set_new_state(mb_goto_voltage);
      }
      else
      {
        robot.u1_req = 0;
        set_new_state(200);
      }
    }
  };

  virtual void state_actions_rules(void)
  {

    // Actions in each state
    if (state == 0)
    { // Robot Stoped
      robot.solenoid_PWM = 0;
      robot.setRobotVW(0, 0, 0);
    }
    else if (state == 1)
    { // Go: Get first box
      robot.solenoid_PWM = 0;
      // robot.followLineLeft(robot.follow_v, robot.follow_k);
      // robot.gotoXYTheta(-0.7225, 0.44, PI/2);
    }
    else if (state == 2)
    { // Box Codes
      if (robot.etapa < 2 && !robot.Box_picked && robot.IRLine_Back.sense_cross)
      {
        robot.setRobotVW(0.03, 0, 0);
        robot.etapa = 1;
      }

      else if (robot.etapa < 3 && !robot.Box_picked && abs(robot.thetae) < 20 * PI / 180 && !robot.IRLine_Back.sense_cross)
      {
        robot.etapa = 2;
        if (robot.xe > 0.03)
        {
          robot.Box_picked = 1;
        }
        else
        {
          robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
          robot.setRobotVW(0.03, robot.vn, robot.w);
        }
      }
      else if (robot.etapa < 4 && robot.Box_picked && abs(robot.thetae) < 10 * PI / 180 && !robot.IRLine_Back.sense_cross)
      {
        robot.etapa = 3;
        robot.setRobotVW(-0.03, robot.vn, robot.w);
      }
      else if (robot.etapa < 5 && robot.Box_picked && (abs(robot.thetae) < 70 * PI / 180 || robot.IRLine_Front.IR_values[2] < 500))
      {
        robot.etapa = 4;
        robot.setRobotVW(0, 0, -0.2);
      }

      else if (robot.etapa < 6 && robot.IRLine_Front.IR_values[2] > 500 && abs(robot.thetae - (-PI / 2)) < 20 * PI / 180 && !robot.IRLine_Back.sense_cross)
      {
        robot.etapa = 5;
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(0.03, robot.vn, robot.w);
      }
      else if (robot.etapa < 7 && (abs(robot.thetae) > 20 * PI / 180 || robot.IRLine_Front.IR_values[2] < 500))
      { // Rodar esquerda
        robot.etapa = 6;
        robot.setRobotVW(0, 0, 0.2);
        robot.time = 0;
      }

      else if (robot.etapa < 8 && (robot.IRLine_Front.IR_total > 500) && robot.time < 7)
      { // Ir para fente
        robot.etapa = 7;
        robot.time += 0.04;
        robot.setRobotVW(0.03, 0, 0);
      }
      else if (robot.etapa < 9 && (robot.ajustar_a > 0 || robot.ajustar_vn > 0))
      { // equilibrar
        robot.etapa = 8;
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(0, robot.vn, robot.w);
      }
      else if (robot.etapa < 10 && !robot.IRLine_Back.sense_cross)
      { // voltar à linha
        robot.etapa = 9;
        robot.setRobotVW(-0.03, robot.vn, robot.w);
      }

      else
      {
        robot.etapa = 15;
        robot.setRobotVW(0, 0, 0);
      }
    }
    else if (state == 3)
    { // Go back with the box

      robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
    }
    else if (state == 4)
    { // Turn arround
      robot.solenoid_PWM = 180;
      robot.setRobotVW(0, 0, 2.5);
    }
    else if (state == 5)
    { // long travel to the box final destination
      robot.solenoid_PWM = 180;
      robot.followLineRight(robot.follow_v, robot.follow_k);
    }
    else if (state == 6)
    { // Advance a little then turn to place the box
      robot.solenoid_PWM = 180;
      robot.setRobotVW(0.05, 0, -2);
    }
    else if (state == 7)
    {
      robot.solenoid_PWM = 180;
      robot.followLineRight(robot.follow_v, robot.follow_k);
    }
    else if (state == 8)
    { // Drop the box and go back
      robot.solenoid_PWM = 0;
      if (tis < 2.0)
        robot.setRobotVW(-0.1, 0, 0); // Dirty hack: a substate in a state
      else
        robot.setRobotVW(0, 0, 0);
    }
    else if (state == 10)
    { // Test
      robot.setRobotVW(0.1, 0, 0);
    }
    else if (state == 100)
    { // Control Stop
      robot.v_req = 0;
      robot.w_req = 0;
    }
    else if (state == 101)
    { // Option for remote control
      robot.control_mode = cm_kinematics;
      robot.setRobotVW(robot.v, robot.vn, robot.w);
    }
    else if (state == 102)
    { // Option for remote PID control
      robot.control_mode = cm_pid;

    } /*else if (state == 199) {
      robot.v_req = 0.1;   // Simple line follwower
      robot.w_req = 4 * robot.IRLine.IR_values[4] / 1024.0
                  + 2 * robot.IRLine.IR_values[3] / 1024.0
                  - 2 * robot.IRLine.IR_values[1] / 1024.0
                  - 4 * robot.IRLine.IR_values[0] / 1024.0;

    }*/
    else if (state == 200)
    { // Direct stop
      robot.control_mode = cm_voltage;
      robot.u1 = 0;
      robot.u2 = 0;
      robot.u3 = 0;
      robot.u4 = 0;
    }
    else if (state == 201)
    { // Option for remote tests
      robot.control_mode = cm_voltage;
      robot.u1 = robot.u1_req;
      robot.u2 = robot.u2_req;
      robot.u3 = robot.u3_req;
      robot.u4 = robot.u4_req;
    }
    else if (state == 202)
    {
      robot.control_mode = cm_voltage;
      robot.u1 = robot.u1_req;
      robot.u2 = robot.u2_req;
      robot.u3 = robot.u3_req;
      robot.u4 = robot.u4_req;
    }
    else if (state == 203)
    {
      robot.send_command("err", "state 203");
      robot.send_command("msg", "state 203");
    }
    else if (state == 300)
    {
      robot.control_mode = cm_pos;
    }
    else if (state == 301)
    {
      robot.FollowLine(0, 0, 0, 0.44, 0);
    }
    else if (state == 302)
    {
      robot.followLineRight(robot.follow_v, robot.follow_k);
    }
    else if (state == mb_heat_motor)
    {
      robot.control_mode = cm_voltage;
      robot.u1_req = motor_bench.warm_up_voltage;
      robot.u2_req = 0;
    }
    else if (state == mb_goto_voltage)
    {
      robot.control_mode = cm_voltage;
    }
    else if (state == mb_acc_measure)
    {
      robot.send_command("mbn", motor_bench.n);
      robot.control_mode = cm_voltage;
      motor_bench.acc_i += robot.i_sense;
      motor_bench.acc_w += robot.w1e;
      motor_bench.n += 1;
    }
    else if (state == 1000)
    { // Set Theta
      traj.set_theta();
    }
  };
};

main_fsm_t main_fsm;

class LED_fsm_t : public state_machine_t
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
  };
};

LED_fsm_t LED_fsm;

void init_control(robot_t &robot)
{
  robot.pfsm = &main_fsm;
  main_fsm.set_new_state(200);
  main_fsm.update_state();
  state_machines.register_state_machine(&main_fsm);
  state_machines.register_state_machine(&LED_fsm);
}

void control(robot_t &robot)
{
  robot.control_mode = cm_kinematics;

  state_machines.step();
}