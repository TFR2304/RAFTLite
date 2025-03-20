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
#define VEL_LIN_NOM 0.1 // 0.2
#define VEL_ANG_NOM 0.2 // 0.5
#define DIST_NEW_POSE 0.02
#define THETA_NEW_POSE 0.02

#define K_ERROR_LIN 10

#define TOL_TH_LOC 0.524 // 30 degres
#define TOL_DIST_LOC 0.05

#define D12 0.16
#define D2z -0.01

// State Machine states
#define Go_Forward 1
#define De_Accel_Lin 2
#define Stop_Lin 3

#define Rotation 1
#define De_Accel_Rot 2
#define Stop_Rot 3

// Used to find fastest rotation
#define RotateRight 1
#define RotateLeft -1

int state_Lin = Stop_Lin;
int state_Rot = Stop_Rot;

robot_t robot;

template <typename T>
int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

robot_t::robot_t()
{
  stoped = false;
  wheel_dist = 0.1925;
  wheel_radius = 0.065 / 2;
  L1_L2 = 0.54;
  dv_max = 5;
  dw_max = 10;
  dt = 0.04;

  p1e = 0;

  // APM
  // follow_k = -0.15;
  follow_k = 0.001;
  // follow_v = 0.20;
  follow_v = 0.10;

  i_lambda = 0;
  led = 0;

  pchannels = NULL;
  pfsm = NULL;
}

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  // w1e = enc1 * (TWO_PI / (5 * 64.0 * 2.0 * 1920.0));//missing gear ratio for the arm and the ferris wheel
  // w2e = enc2 * (TWO_PI / (5 * 64.0 * 2.0 * 1920.0));

  // p1e += w1e*dt;

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
  w1e = robot.enc1 * (TWO_PI / (64 * 2.0 * 1920.0));
  w2e = robot.enc2 * (TWO_PI / (64 * 2.0 * 1920.0));
  w3e = robot.enc3 * (TWO_PI / (64 * 2.0 * 1920.0));
  w4e = robot.enc4 * (TWO_PI / (64 * 2.0 * 1920.0));

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
  ve = (v3e + v4e + v1e + v2e) / 4.0;          //(v1e + v2e + v3e + v4e) / 4.0;
  vne = (-v3e + v4e + v1e - v2e) / 4.0;        //(-v1e + v2e + v3e - v4e) / 4.0;
  we = (-v3e + v4e - v1e + v2e) / (4 * L1_L2); //(-v1e + v2e - v3e + v4e) / (2*L1_L2);

  // Estimate the distance and the turn angle
  ds = ve * dt;
  dns = vne * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta / 2);
  ye += ds * sin(thetae + dtheta / 2);
  /* Nao mexer nesta parte
    if (fabs(dtheta) < 0.000175)
    {
      xe += ds * cos(thetae) - dns * sin(thetae);
      ye += ds * sin(thetae) + dns * cos(thetae);
    }
    else
    {
      xe += (ds * sin(dtheta) + dns * (cos(dtheta) - 1)) * cos(thetae + dtheta / 2) / dtheta - (ds * (1 - cos(dtheta)) + dns * sin(dtheta)) * sin(thetae + dtheta / 2) / dtheta;
      ye += (ds * sin(dtheta) + dns * (cos(dtheta) - 1)) * sin(thetae + dtheta / 2) / dtheta + (ds * (1 - cos(dtheta)) + dns * sin(dtheta)) * cos(thetae + dtheta / 2) / dtheta;
    }
      */
  thetae += dtheta;

  // Relative displacement
  rel_s += ds;
  rel_ns += dns;
  rel_theta += dtheta;
}

void robot_t::localization(void)
{
  float alfa;

  // Line Crosses must be first to calculate variable "total" and "last_total"
  if (robot.IRLine_Front.SenseIRLineCrosses() == true)
  {
    // Serial.println("Found");

    if ((abs(xe - (-0.724)) < TOL_DIST_LOC) && (abs(thetae) < TOL_TH_LOC / 3))
    {
      // CAUTION! difference between marks must be greater than TOL_DIST_LOC
      if (abs(ye - (-0.265)) < TOL_DIST_LOC)
        ye = -0.265;
      if (abs(ye - (-0.237)) < TOL_DIST_LOC)
        ye = -0.237;
      if (abs(ye - (-0.116)) < TOL_DIST_LOC)
        ye = -0.116;
      if (abs(ye - (-0.083)) < TOL_DIST_LOC)
        ye = -0.083;
      if (abs(ye - 0.255) < TOL_DIST_LOC)
        ye = 0.255;
      if (abs(ye - 0.285) < TOL_DIST_LOC)
        ye = 0.285;
    }
  }

  if (robot.IRLine_Back.SenseIRLineCrosses() == true)
  {
    if ((abs(xe - (-0.724)) < TOL_DIST_LOC) && (abs(thetae - PI / 2) < TOL_TH_LOC / 3))
    {
      // CAUTION! difference between marks must be greater than TOL_DIST_LOC
      if (abs(ye - (-0.308)) < TOL_DIST_LOC)
        ye = -0.308;
      if (abs(ye - (-0.279)) < TOL_DIST_LOC)
        ye = -0.279;
      if (abs(ye - (-0.083)) < TOL_DIST_LOC)
        ye = -0.083;
      if (abs(ye - (-0.057)) < TOL_DIST_LOC)
        ye = -0.057;
      if (abs(ye - 0.063) < TOL_DIST_LOC)
        ye = 0.063;
      if (abs(ye - 0.096) < TOL_DIST_LOC)
        ye = 0.096;
    }
  }

  robot.IRLine_Back.dist_center = 999; // 999 - no line detection
  if (robot.IRLine_Back.total < robot.IRLine_Back.cross_total_tresh)
  { // near crosses! Invalid Edges!
    if (robot.IRLine_Back.calcIRLineEdgeLeft() == true)
    {
      robot.IRLine_Back.dist_center = robot.IRLine_Back.pos_left;
      if (robot.IRLine_Back.calcIRLineEdgeRight() == true)
        robot.IRLine_Back.dist_center = (robot.IRLine_Back.pos_left + robot.IRLine_Back.pos_right) / 2;
    }
    else
    {
      if (robot.IRLine_Back.calcIRLineEdgeRight() == true)
        robot.IRLine_Back.dist_center = robot.IRLine_Back.pos_right;
    }
  }

  robot.IRLine_Front.dist_center = 999; // 999 - no line detection
  if (robot.IRLine_Front.total < robot.IRLine_Front.cross_total_tresh)
  { // near crosses! Invalid Edges!
    if (robot.IRLine_Front.calcIRLineEdgeLeft() == true)
    {
      robot.IRLine_Front.dist_center = robot.IRLine_Front.pos_left;
      if (robot.IRLine_Front.calcIRLineEdgeRight() == true)
        robot.IRLine_Front.dist_center = (robot.IRLine_Front.pos_left + robot.IRLine_Front.pos_right) / 2;
    }
    else
    {
      if (robot.IRLine_Front.calcIRLineEdgeRight() == true)
        robot.IRLine_Front.dist_center = robot.IRLine_Front.pos_right;
    }
  }

  if ((robot.IRLine_Front.dist_center != 999) && (robot.IRLine_Back.dist_center != 999))
  { // detection in both sensors
    alfa = atan((robot.IRLine_Front.dist_center - robot.IRLine_Back.dist_center) / D12);

    if (abs(thetae - PI / 2) < TOL_TH_LOC)
    {
      if (abs(xe - (-0.724)) < TOL_DIST_LOC)
      {
        xe = -0.724 + (robot.IRLine_Front.dist_center + tan(alfa) * (-(D12 + D2z))) * cos(alfa);
        thetae = PI / 2 - alfa;
      }
    }
  }
}

void robot_t::setRobotVW(float Vnom, float VNnom, float Wnom)
{
  v = Vnom;
  vn = VNnom;
  w = Wnom;
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
  if (control_mode == cm_voltage)
  {
    u1 = u1_req;
    u2 = u2_req;
    u3 = u3_req;
    u4 = u4_req;
    return;
  }
  else if (control_mode == cm_pid)
  {
    w1ref = w1_req;
    w2ref = w2_req;
    w3ref = w3_req;
    w4ref = w4_req;
  }
  else if (control_mode == cm_kinematics)
  {
    v1ref = v - vn - w * L1_L2 / 2;
    v2ref = v + vn + w * L1_L2 / 2;
    v3ref = v + vn - w * L1_L2 / 2;
    v4ref = v - vn + w * L1_L2 / 2;

    w1ref = (v1ref / wheel_radius);
    w2ref = v2ref / wheel_radius;
    w3ref = (v3ref / wheel_radius);
    w4ref = v4ref / wheel_radius;
  }
  else if (control_mode == cm_pos)
  {
    p1ref = p1_req;
    PID[0].last_pe = PID[0].pos_error;
    PID[0].pos_error = p1ref - p1e;
    w1ref = PID[0].ppars->Kc_p * PID[0].pos_error + PID[0].ppars->Kd_p * (PID[0].pos_error - PID[0].last_pe) / PID[0].ppars->dt;

    if ((abs(PID[0].pos_error) < 0.005) && (fabs(w1ref < 0.015)))
      u1 = 0; // Cycle Oscillation Limiter
    else
      u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) * PID[0].ppars->dead_zone;
  }

  if (control_mode != cm_pos)
  {
    if (w1ref != 0)
      u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) * PID[0].ppars->dead_zone;
    else
    {
      u1 = 0;
      PID[0].Se = 0;
      PID[0].y_ref = 0;
    }

    if (w2ref != 0)
      u2 = PID[1].calc(w2ref, w2e) + sign(w2ref) * PID[1].ppars->dead_zone;
    else
    {
      u2 = 0;
      PID[1].Se = 0;
      PID[1].y_ref = 0;
    }

    if (w3ref != 0)
      u3 = PID[2].calc(w3ref, w3e) + sign(w3ref) * PID[2].ppars->dead_zone;
    else
    {
      u3 = 0;
      PID[2].Se = 0;
      PID[2].y_ref = 0;
    }

    if (w4ref != 0)
      u4 = PID[3].calc(w4ref, w4e) + sign(w4ref) * PID[3].ppars->dead_zone;
    else
    {
      u4 = 0;
      PID[3].Se = 0;
      PID[3].y_ref = 0;
    }
  }
  // PWM_2 = u2 / battery_voltage * 255;
  // PWM_1 = u1 / battery_voltage * 255;
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
  w_req = -K * IRLine_Front.pos_left;
  ;
  v_req = Vnom;
}

float robot_t::dist(float xd, float yd)
{
  return sqrt(pow(xd, 2) + pow(yd, 2));
}

void robot_t::dist2Line(float xe, float ye, float xi, float yi, float xf, float yf, float &xr, float &yr)
{
  float ux = (xf - xi) / dist(xf - xi, yf - yi);
  float uy = (yf - yi) / dist(xf - xi, yf - yi);

  float k = (xe * uy - ye * ux - xi * uy + yi * ux) / (pow(ux, 2) + pow(uy, 2));

  xr = xe - k * uy;
  yr = ye + k * ux;
}

void robot_t::FollowLine(float xi, float yi, float xf, float yf, float thf)
{
  float error_dist = dist(xf - xe, yf - ye);
  float xr, yr;
  dist2Line(xe, ye, xi, yi, xf, yf, xr, yr);
  float alfa = atan2(yf - ye, xf - xe);

  float error_final_rot = normalize_angle(thf - thetae);

  // Find fastest rotation
  int rotate_to_final = 0;
  if (error_final_rot > 0.0)
    rotate_to_final = RotateRight;
  else
    rotate_to_final = RotateLeft;

  switch (state_Lin)
  {
  case Go_Forward:
    if (error_dist < TOL_FINDIST)
      state_Lin = Stop_Lin;
    else if (error_dist < DIST_DA)
      state_Lin = De_Accel_Lin;
    break;
  case De_Accel_Lin:
    if (error_dist < TOL_FINDIST)
      state_Lin = Stop_Lin;
    else if (error_dist > DIST_NEW_POSE)
      state_Lin = Go_Forward;
    break;
  case Stop_Lin:
    if (error_dist > DIST_NEW_POSE)
      state_Lin = Go_Forward;
    break;
  default:
    state_Lin = Stop_Lin;
  }

  // Transitions Rotation
  switch (state_Rot)
  {
  case Rotation:
    if (abs(error_final_rot) < TOL_FINTHETA)
      state_Rot = Stop_Rot;
    else if (abs(error_final_rot) < THETA_DA)
      state_Rot = De_Accel_Rot;
    break;
  case De_Accel_Rot:
    if (abs(error_final_rot) < TOL_FINTHETA)
      state_Rot = Stop_Rot;
    else if (abs(error_final_rot) > DIST_NEW_POSE)
      state_Rot = Rotation;
    break;
  case Stop_Rot:
    if (abs(error_final_rot) > THETA_NEW_POSE)
      state_Rot = Rotation;
    break;
  default:
    state_Rot = Stop_Rot;
  }

  // Outputs Linear Velocity
  double VlinX, VlinY;
  VlinX = cos(alfa) + K_ERROR_LIN * (xr - xe);
  VlinY = sin(alfa) + K_ERROR_LIN * (yr - ye);
  VlinX = VlinX / sqrt(pow(VlinX, 2) + pow(VlinY, 2));
  VlinY = VlinY / sqrt(pow(VlinX, 2) + pow(VlinY, 2));
  switch (state_Lin)
  {
  case Go_Forward:
    v_req = VEL_LIN_NOM * (VlinX * cos(thetae) + VlinY * sin(thetae));
    vn_req = VEL_LIN_NOM * (-VlinX * sin(thetae) + VlinY * cos(thetae));
    break;
  case De_Accel_Lin:
    v_req = (VEL_LIN_NOM / 3) * (VlinX * cos(thetae) + VlinY * sin(thetae));
    vn_req = (VEL_LIN_NOM / 3) * (-VlinX * sin(thetae) + VlinY * cos(thetae));
    break;
  case Stop_Lin:
    v_req = 0.0;
    vn_req = 0.0;
    break;
  default:
    v_req = 0.0;
    vn_req = 0.0;
  }

  // Outputs Rotation
  switch (state_Rot)
  {
  case Rotation:
    w_req = VEL_ANG_NOM * rotate_to_final;
    break;
  case De_Accel_Rot:
    w_req = (VEL_ANG_NOM / 3) * rotate_to_final;
    break;
  case Stop_Lin:
    w_req = 0.0;
    break;
  default:
    w_req = 0.0;
  }
}

void robot_t::gotoXYTheta(float xf, float yf, float thf)
{
  float ang_target = atan2(yf - ye, xf - xe);
  float error_dist = sqrt(sqr(xf - xe) + sqr(yf - ye));
  float error_finalrot = normalize_angle(thf - thetae);
  if (error_dist < TOL_FINDIST)
  {
    v_req = 0;
    vn_req = 0;
    // at_target_pos = true;
  }
  else if (error_dist < DIST_DA)
  {
    v_req = (VEL_LIN_NOM / 3) * cos(ang_target - thetae);
    vn_req = (VEL_LIN_NOM / 3) * sin(ang_target - thetae);
  }
  else
  {
    v_req = VEL_LIN_NOM * cos(ang_target - thetae);
    vn_req = VEL_LIN_NOM * sin(ang_target - thetae);
  }
  if (abs(error_finalrot) < TOL_FINTHETA)
  {
    w_req = 0;
    // at_target_ang = true;
  }
  else if (abs(error_finalrot) < THETA_DA)
  {
    w_req = signal(error_finalrot) * VEL_ANG_NOM / 3;
  }
  else
  {
    w_req = signal(error_finalrot) * VEL_ANG_NOM;
  }
  debug = error_finalrot;
}

void robot_t::send_command(const char *command, float par)
{
  if (pchannels)
    pchannels->send_command(command, par);
}

void robot_t::send_command(const char *command, const char *par)
{
  if (pchannels)
    pchannels->send_command(command, par);
}