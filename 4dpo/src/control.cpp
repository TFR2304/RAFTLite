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
    else if (state == 10 && robot.xe > 0.88)
    {
      set_new_state(32);
    }
    else if (state == 11 && tis > 2.0)
    {
      if (abs(robot.thetae - (-3.14)) <= 20 * PI / 180)
      {
        set_new_state(0);
      }
    }
    else if (state == 20 && robot.xe > 0.765)
    {
      robot.pick_ok = 1.0;
      set_new_state(21);
    }
    else if (state == 21 && tis > 3 && robot.pick_done == 1.0)
    {
      robot.pick_ok = 0.0;
      robot.pick_done = 0.0;
      set_new_state(33);
    }
    else if (state == 29 && tis > 3 && robot.IRLine_Front.IR_values[3] > 600)
    {

      set_new_state(30);
    }
    else if (state == 30 && !(robot.ajustar_a == 2 && robot.ajustar_vn == 2))
    {
      set_new_state(31);
    }
    else if (state == 31 && robot.ajustar_a == 0 && robot.ajustar_vn == 0)
    {
      set_new_state(32);
    }
    else if (state == 31 && robot.ajustar_a == 2 && robot.ajustar_vn == 2)
    {
      set_new_state(30);
    }

    else if (state == 32 && robot.IRLine_Back.sense_cross)
    {
      set_new_state(20);
    }
    else if (state == 33 && robot.IRLine_Back.sense_cross)
    {
      set_new_state(29);
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
    }
    else if (state == 2)
    { // Box Codes
      if (!robot.Box_picked && robot.IRLine_Back.sense_cross)
      {

        robot.setRobotVW(0.03, 0, 0);
        robot.etapa = 1;
        robot.xe = 0;
      }

      else if (!robot.Box_picked && abs(robot.thetae) < 20 * PI / 180 && !robot.IRLine_Back.sense_cross)
      {
        robot.etapa = 2;
        if (robot.xe > 0.02)
        {
          robot.Box_picked = 1;
        }
        else
        {
          robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
          robot.setRobotVW(0.02, robot.vn, robot.w);
        }
      }
      else if (robot.Box_picked && !robot.IRLine_Back.sense_cross)
      { // voltar atrás
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(-0.03, robot.vn, robot.w);
        robot.time = 0;
      }
      else if (robot.time < 2 || robot.IRLine_Front.IR_values[2] < 500)
      {
        robot.time += 0.04;
        robot.setRobotVW(0, -0.04, -0.0196);
      }
      else if (robot.ajustar_a == 1 || robot.ajustar_vn == 1)
      {
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(0, robot.vn, robot.w);
      }
      else if (!robot.IRLine_Back.sense_cross)
      {
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(-0.03, robot.vn, robot.w);
      }
      else
      {
        robot.Box_picked = 0;
      }
    }
    else if (state == 3)
    { // Go back with the box

      robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
      robot.setRobotVW(0, robot.vn, robot.w);
    }
    else if (state == 4)
    { // Pick Boxes
      if (robot.IRLine_Back.sense_cross)
      {
        robot.setRobotVW(0.03, 0, 0);
        robot.xe = 0;
      }
      else if (robot.xe < 0.01)
      {
        robot.setRobotVW(0.03, robot.vn, robot.w);
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
      }
      else if (robot.time < 10 || robot.IRLine_Front.IR_values[2] < 500)
      {
        robot.time += 0.04;
        if (robot.thetae < 25 * PI / 180)
        {
          robot.setRobotVW(0, 0, 0.08);
        }
        else
        {

          robot.setRobotVW(0, -0.05, 0);
        }
      }
      else if (robot.ajustar_a != 0 || robot.ajustar_vn != 0)
      {

        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(0, robot.vn, robot.w);
      }
      else if (robot.ajustar_a == 0 && robot.ajustar_vn == 0)
      {
        robot.setRobotVW(0, robot.vn, robot.w);
      }
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
      if (robot.xe < 0.85)
      {
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(0.15, robot.vn, robot.w);
      }
      else
      {
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(0.03, robot.vn, robot.w);
      }
    }
    else if (state == 11)
    { // Test
      if (dist(robot.xe, robot.ye, float(0.735), float(1.500)) >= 0.05)
      {
        robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
        robot.setRobotVW(0.06, robot.vn, robot.w);
      }
      else
      {
        robot.setRobotVW(0, 0, -0.2);
      }
    }
    else if (state == 20)
    { // Ir para pegar caixa

      robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
      robot.setRobotVW(0.03, robot.vn, robot.w);
    }
    else if (state == 21)
    { // Pegar Na caixa
      robot.setRobotVW(0, 0, 0);
    }
    else if (state == 22)
    {
      robot.setRobotVW(0, 0, 0);
    }

    else if (state == 29)
    { // mover para o lado
      robot.setRobotVW(0, -0.04, -0.0195);
    }
    else if (state == 30)
    { // mover para frente
      robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
      robot.setRobotVW(0.01, 0, 0);
    }
    else if (state == 31)
    { // Endireitar Na caixa
      robot.Correct_Line(robot.IRLine_Back.k_vn, robot.IRLine_Back.k_alfa);
      robot.setRobotVW(0, robot.vn, robot.w);
    }
    else if (state == 32)
    { // Recuar até cruzamento
      robot.setRobotVW(-0.01, 0, 0);
      robot.xe=0.735;
    }
    else if (state == 33)
    { // Recuar até cruzamento para depois rodar para o lado
      robot.setRobotVW(-0.01, 0, 0);
      robot.xe = 0.735;
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