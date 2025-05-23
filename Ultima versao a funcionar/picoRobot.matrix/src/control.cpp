
#include <Arduino.h>
#include "robot.h"
#include "state_machines.h"
#include "trajectories.h"


class main_fsm_t: public state_machine_t
{
  virtual void next_state_rules(void) 
  {
    // Rules for the state evolution
     if(state == 0 && robot.tof_dist > 0.10 && robot.prev_tof_dist < 0.05 && tis > 1.0) {
      robot.rel_s = 0;
      set_new_state(1);

    } else if (state == 1 && robot.TouchSwitch) {
      robot.at_destin = false;
      set_new_state(2);

    } else if (state == 2 && robot.at_destin) {
      robot.at_destin = false;
      set_new_state(3);

    } else if (state == 3) {
      //robot.rel_theta = 0;
      //set_new_state(4);

    } else if (state == 4 && robot.rel_theta > radians(170)) {
      set_new_state(5);
      robot.IRLine_Front.crosses = 0;

    } else if (state == 5 && robot.IRLine_Front.crosses >= 5) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      set_new_state(6);

    } else if (state == 6 && robot.rel_theta < radians(-70) && robot.IRLine_Front.total > 1500) {
      set_new_state(7);

    } else if (state == 7 && tis > 2.0) {
      robot.IRLine_Front.crosses = 0;
      set_new_state(8);
    
    } else if (state == 202 && tis > 2.0) {
      robot.IRLine_Front.crosses = 0;
      set_new_state(200);

    } else if (state == 203 && actions_count > 0) {
      set_new_state(prev_state);  
    }

  };


  virtual void state_actions_rules(void)
  {
 // Actions in each state
    if (state == 0) {         // Robot Stoped            
      robot.solenoid_PWM = 0;
      robot.setRobotVW(0, 0, 0);

    } else if (state == 1) {  // Go: Get first box
      robot.solenoid_PWM = 0;
      //robot.followLineLeft(robot.follow_v, robot.follow_k);
      robot.gotoXYTheta(-0.7225, 0.44, PI/2);

    } else if (state == 2) { // Turn Solenoid On and Get the Box
      robot.solenoid_PWM = 180;
      //robot.followLineLeft(robot.follow_v, robot.follow_k);
      robot.gotoXYTheta(-0.7225, 0.34, PI/2);
    
    } else if (state == 3) {  // Go back with the box
      //robot.solenoid_PWM = 180;
      //robot.setRobotVW(-0.1, 0, 0);
      robot.gotoXYTheta(-0.6, 0.24, 0);
      
    } else if (state == 4) {  // Turn arround
      robot.solenoid_PWM = 180;
      robot.setRobotVW(0, 0, 2.5);
      
    } else if (state == 5) {  // long travel to the box final destination
      robot.solenoid_PWM = 180;
      robot.followLineRight(robot.follow_v, robot.follow_k);
      
    } else if (state == 6) {  // Advance a little then turn to place the box
      robot.solenoid_PWM = 180;
      robot.setRobotVW(0.05, 0, -2);
   

    } else if (state == 7) {  
      robot.solenoid_PWM = 180;
      robot.followLineRight(robot.follow_v, robot.follow_k); 

    } else if (state == 8) { // Drop the box and go back
      robot.solenoid_PWM = 0;
      if (tis < 2.0) robot.setRobotVW(-0.1, 0, 0);  // Dirty hack: a substate in a state
      else robot.setRobotVW(0, 0, 0);
    
    } else if (state == 10) { // Test
      robot.setRobotVW(0.1, 0, 0);      

    } else if (state == 100) {  // Control Stop
      robot.v_req = 0;
      robot.w_req = 0;

    } else if (state == 101) {  // Option for remote control
      robot.control_mode = cm_kinematics;
      robot.setRobotVW(robot.v_req, robot.vn_req, robot.w_req);

    } else if (state == 102) {  // Option for remote PID control
      robot.control_mode = cm_pid;
      

    } else if (state == 199) {
      /*robot.v_req = 0.1;   // Simple line follwower
      robot.w_req = 4 * IRLine.IR_values[4] / 1024.0 
                  + 2 * IRLine.IR_values[3] / 1024.0
                  - 2 * IRLine.IR_values[1] / 1024.0
                  - 4 * IRLine.IR_values[0] / 1024.0;*/

    } else if (state == 200) {  // Direct stop
      robot.control_mode = cm_voltage;
      robot.u1 = 0;
      robot.u2 = 0;

    } else if (state == 201) {  // Option for remote tests
      robot.control_mode = cm_voltage;
      robot.u1 = robot.u1_req;
      robot.u2 = robot.u2_req;
    
    } else if (state == 202) {
      robot.control_mode = cm_voltage;
      robot.u1 = robot.u1_req;
      robot.u2 = robot.u2_req;      

    } else if (state == 203) {
      robot.send_command("err", "state 203");
      robot.send_command("msg", "state 203");
    
    } else if (state == 1000) {  // Set Theta
      traj.set_theta();
    }    
  };  
};

main_fsm_t main_fsm;


class LED_fsm_t: public state_machine_t
{
  virtual void next_state_rules(void) 
  {
    // Rules for the state evolution
     if(state == 0 && tis > 0.1) {
      set_new_state(1);

    } else if (state == 1 && tis > 0.2) {
      set_new_state(2);

    } else if(state == 2 && tis > 0.2) {
      set_new_state(3);

    } else if(state == 3 && tis > 0.6) {
      set_new_state(0);
    }
  };


  virtual void state_actions_rules(void)
  {
    // Actions in each state
    if (state == 0) {        // LED on
      robot.led = 1;

    } else if (state == 1) { // LED off
      robot.led = 0;

    } else if (state == 2) { // LED on 
      robot.led = 1;
    
    } else if (state == 3) { // LED off
      robot.led = 0;
    }      
  };  
};

LED_fsm_t LED_fsm;

void init_control(robot_t& robot)
{
  robot.pfsm = &main_fsm;
  main_fsm.set_new_state(200);
  main_fsm.update_state();
  state_machines.register_state_machine(&main_fsm);
  state_machines.register_state_machine(&LED_fsm); 
}

void control(robot_t& robot)
{
  robot.control_mode = cm_kinematics;

  state_machines.step();
}