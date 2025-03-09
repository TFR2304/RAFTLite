/* Copyright (c) 2021  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include "robot.h"
#include <math.h>


robot_t robot;

template <typename T> int sign(T val) 
{
  return (T(0) < val) - (val < T(0));
}

robot_t::robot_t()
{
  stoped = false;
  wheel_dist = 0.01925;
  wheel_radius = 0.065/2;
  L1_L2 = 0.54;
  dv_max = 5;
  dw_max = 10;
  dt = 0.04;

  p1e = 0;

   // APM
    //follow_k = -0.15;
    follow_k = 0.001;
    //follow_v = 0.20;
    follow_v = 0.10;

  i_lambda = 0;
  led = 0;

  pchannels = NULL;
  pfsm = NULL;
}

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  //w1e = enc1 * (TWO_PI / (5 * 64.0 * 2.0 * 1920.0));//missing gear ratio for the arm and the ferris wheel 
  //w2e = enc2 * (TWO_PI / (5 * 64.0 * 2.0 * 1920.0));

  //p1e += w1e*dt;

  /*v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;*/
  w1e = robot.enc1 * (TWO_PI / (64* 2.0 * 1920.0));
  w2e = robot.enc2 * (TWO_PI / (64* 2.0 * 1920.0));
  w3e = robot.enc3 * (TWO_PI / (64* 2.0 * 1920.0));
  w4e = robot.enc4 * (TWO_PI / (64* 2.0 * 1920.0));  
  
    /*
    w1e = robot.enc1;
    w2e = robot.enc2;
    w3e = robot.enc3;
    w4e = robot.enc4;  
    */
  
    v1e = w1e * wheel_radius;
    v2e = w2e * wheel_radius;
    v3e = w3e * wheel_radius;
    v4e = w4e * wheel_radius;
  
    // Estimate robot speed
    ve = (v1e + v2e + v3e + v4e) / 4.0;
    vne = (-v1e + v2e + v3e - v4e) / 4.0;
    we = (-v1e + v2e - v3e + v4e) / (2*L1_L2);
    
    // Estimate the distance and the turn angle
    ds = ve * dt;
    dns = vne * dt;
    dtheta = we * dt;
  
    // Estimate pose
    xe += ds * cos(thetae + dtheta/2);
    ye += ds * sin(thetae + dtheta/2);
    
   if (fabs(dtheta) < 0.000175) {
      xe += ds*cos(thetae)-dns*sin(thetae);
      ye += ds*sin(thetae)+dns*cos(thetae);
    } else {
      xe += (ds*sin(dtheta)+dns*(cos(dtheta)-1))*cos(thetae + dtheta/2)/dtheta
                   - (ds*(1 - cos(dtheta))+dns*sin(dtheta))*sin(thetae+dtheta/2)/dtheta; 
      ye += (ds*sin(dtheta)+dns*(cos(dtheta)-1))*sin(thetae + dtheta/2)/dtheta
                   + (ds*(1 - cos(dtheta))+dns*sin(dtheta))*cos(thetae + dtheta/2)/dtheta;     
    }
    thetae += dtheta;
    
    // Relative displacement
    rel_s += ds;
    rel_ns += dns;
    rel_theta += dtheta;
}


void robot_t::setRobotVW(float Vnom, float Wnom)
{
  v_req = Vnom;
  w_req = Wnom;
}


void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}


void robot_t::calcMotorsVoltage(void)
{
  if (control_mode == cm_voltage) {
    u1 = u1_req;
    u2 = u2_req;
    u3 = u3_req;
    u4 = u4_req;  
    return;

  } else if (control_mode == cm_pid) {
    w1ref = w1_req;
    w2ref = w2_req;
    w3ref = w3_req;
    w4ref = w4_req;
    

  } else if (control_mode == cm_kinematics) {
    v1ref = v + w * wheel_dist / 2;
    v2ref = v - w * wheel_dist / 2; 
    
    w1ref = v1ref / wheel_radius;
    w2ref = v2ref / wheel_radius;    
  } else if (control_mode == cm_pos) {
    p1ref = p1_req;
    PID[0].last_pe = PID[0].pos_error;
    PID[0].pos_error = p1ref-p1e;
    w1ref = PID[0].ppars->Kc_p*PID[0].pos_error + PID[0].ppars->Kd_p*(PID[0].pos_error - PID[0].last_pe)/PID[0].ppars->dt;

    if((abs(PID[0].pos_error)<0.005)&&(fabs(w1ref < 0.015))) u1=0; //Cycle Oscillation Limiter
    else u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) *  PID[0].ppars->dead_zone; 
  }
    else if(control_mode == cm_IR){
      followLineRight(1.5, follow_k);

    }

  if (control_mode != cm_pos) {
    if (w1ref != 0) u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) *  PID[0].ppars->dead_zone;
    else {
      u1 = 0;
      PID[0].Se = 0;
      PID[0].y_ref = 0;
    }

    if (w2ref != 0) u2 = PID[1].calc(w2ref, w2e) + sign(w2ref) *  PID[1].ppars->dead_zone;
    else {
      u2 = 0;      
      PID[1].Se = 0;
      PID[1].y_ref = 0;
    }

    
    if (w3ref != 0) 
      u3 = PID[2].calc(w3ref, w3e) + sign(w3ref) *  PID[2].ppars->dead_zone;
    else {
      u3 = 0;      
      PID[2].Se = 0;
      PID[2].y_ref = 0;
    }
  
    if (w4ref != 0) 
      u4 = PID[3].calc(w4ref, w4e) + sign(w4ref) *  PID[3].ppars->dead_zone;
    else {
      u4 = 0;      
      PID[3].Se = 0;
      PID[3].y_ref = 0;
    }
  }
  //PWM_2 = u2 / battery_voltage * 255;
  //PWM_1 = u1 / battery_voltage * 255;

}


void robot_t::followLineRight(float Vnom, float K)
{
  w_req = K * IRLine.pos_right;
  //w_req = w_req * fabs(w_req);
  //v_req = fmax(0, Vnom - 0.1 * fabs(w_req));
  v_req = Vnom;
}


void robot_t::followLineLeft(float Vnom, float K)
{
  w_req = K * IRLine.pos_left;
  //w_req = w_req * fabs(w_req);
  //v_req = fmax(0, Vnom - 0.1 * fabs(w_req));
  v_req = Vnom;
}

void robot_t::send_command(const char* command, float par)
{
  if (pchannels) pchannels->send_command(command, par);
}

void robot_t::send_command(const char* command, const char* par)
{
  if (pchannels) pchannels->send_command(command, par);
}