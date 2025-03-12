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
#include <trajectories.h>


#define TOL_FINDIST 0.01 
#define DIST_DA 0.05
#define THETA_DA 0.05
#define TOL_FINTHETA 0.01
#define VEL_LIN_NOM 0.2
#define VEL_ANG_NOM 0.5

#define TOL_TH_LOC 0.524 // 30 degres
#define TOL_DIST_LOC 0.05

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

void robot_t::localization(void)
{
float alfa;

robot.IRLine_Back.dist_center = 999; // 999 - no line detection
if (robot.IRLine_Back.calcIRLineEdgeLeft() == true) {
  robot.IRLine_Back.dist_center = robot.IRLine_Back.pos_left;
  if (robot.IRLine_Back.calcIRLineEdgeRight() == true) robot.IRLine_Back.dist_center = (robot.IRLine_Back.pos_left + robot.IRLine_Back.pos_right)/2;
} else {
  if (robot.IRLine_Back.calcIRLineEdgeRight() == true) robot.IRLine_Back.dist_center = robot.IRLine_Back.pos_right;
} 


robot.IRLine_Front.dist_center = 999; // 999 - no line detection
if (robot.IRLine_Front.calcIRLineEdgeLeft() == true) {
  robot.IRLine_Front.dist_center = robot.IRLine_Front.pos_left;
  if (robot.IRLine_Front.calcIRLineEdgeRight() == true) robot.IRLine_Front.dist_center = (robot.IRLine_Front.pos_left + robot.IRLine_Front.pos_right)/2;
} else {
  if (robot.IRLine_Front.calcIRLineEdgeRight() == true) robot.IRLine_Front.dist_center = robot.IRLine_Front.pos_right;
} 

if ((robot.IRLine_Front.dist_center != 999) && (robot.IRLine_Back.dist_center != 999)) {  // detection in both sensors
  alfa = atan((robot.IRLine_Front.dist_center - robot.IRLine_Back.dist_center)/(0.24 +0.28));
  //debug4 = RAD_TO_DEG*alfa;

  if (abs(thetae - PI/2) < 0.524){
    if (abs(xe -(-0.723)) < 0.05) {
        xe = -0.723 + (tan(alfa)*(0.26) + robot.IRLine_Front.dist_center)*cos(alfa);
    }
  }
}

}


void robot_t::setRobotVW(float Vnom, float VNnom, float Wnom)
{
  v_req = Vnom;
  vn_req = VNnom;
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
    v1ref = v - vn - w*L1_L2 / 2;
    v2ref = v + vn + w*L1_L2 / 2;
    v3ref = v + vn - w*L1_L2 / 2;
    v4ref = v - vn + w*L1_L2 / 2;
    
    w1ref = v1ref / wheel_radius;
    w2ref = v2ref / wheel_radius;    
    w3ref = v3ref / wheel_radius;
    w4ref = v4ref / wheel_radius;     
  } else if (control_mode == cm_pos) {
    p1ref = p1_req;
    PID[0].last_pe = PID[0].pos_error;
    PID[0].pos_error = p1ref-p1e;
    w1ref = PID[0].ppars->Kc_p*PID[0].pos_error + PID[0].ppars->Kd_p*(PID[0].pos_error - PID[0].last_pe)/PID[0].ppars->dt;

    if((abs(PID[0].pos_error)<0.005)&&(fabs(w1ref < 0.015))) u1=0; //Cycle Oscillation Limiter
    else u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) *  PID[0].ppars->dead_zone; 
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
  vn_req = 0; 
  w_req = -K * IRLine_Front.pos_right;
  v_req = 0;
}


void robot_t::followLineLeft(float Vnom, float K)
{
  vn_req = 0;
  w_req = -K * IRLine_Front.pos_left;;
  v_req = Vnom;
}

void robot_t::gotoXYTheta(float xf, float yf, float thf)
{
  float ang_target = atan2(yf-ye, xf-xe);
  float error_dist = sqrt(sqr(xf-xe) + sqr(yf-ye));
  float error_finalrot = normalize_angle(thf - thetae);
  if (error_dist < TOL_FINDIST) {
    v_req = 0;
    vn_req = 0;
    //at_target_pos = true;
  } else if (error_dist < DIST_DA) {
      v_req = (VEL_LIN_NOM/3)*cos(ang_target-thetae);
      vn_req = (VEL_LIN_NOM/3)*sin(ang_target-thetae);
  } else {
      v_req = VEL_LIN_NOM*cos(ang_target-thetae);
      vn_req = VEL_LIN_NOM*sin(ang_target-thetae);
  }
  if (abs(error_finalrot) < TOL_FINTHETA) {
    w_req = 0;
    //at_target_ang = true;
  } else if (abs(error_finalrot) < THETA_DA) {
      w_req = signal(error_finalrot)*VEL_ANG_NOM/3;
  } else {
      w_req = signal(error_finalrot)*VEL_ANG_NOM;
  }  
  debug = error_finalrot;
}


void robot_t::send_command(const char* command, float par)
{
  if (pchannels) pchannels->send_command(command, par);
}

void robot_t::send_command(const char* command, const char* par)
{
  if (pchannels) pchannels->send_command(command, par);
}