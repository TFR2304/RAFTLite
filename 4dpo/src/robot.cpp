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
#define D2z -0.02

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
  L1_L2 = 0.56;
  dv_max = 5;
  dw_max = 10;
  dt = 0.04;

  pick_ok = 0;
  pick_done = 0;
  drop_done = 0;
  drop_ok = 0;

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

  // v1e = w1e * wheel_radius;
  // v2e = w2e * wheel_radius;

  // Estimate robot speed
  /*ve = (v1e + v2e) / 2.0;
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

  v1e = -w1e * wheel_radius;
  v2e = -w2e * wheel_radius;
  v3e = -w3e * wheel_radius;
  v4e = -w4e * wheel_radius;

  // Estimate robot speed
  ve = (v3e - v4e + v1e - v2e) / 4.0;          //(v1e + v2e + v3e + v4e) / 4.0;
  vne = -(v3e + v4e - v1e - v2e) / 4.0;        //(-v1e + v2e + v3e - v4e) / 4.0;
  we = (-v3e - v4e - v1e - v2e) / (2 * L1_L2); //(-v1e + v2e - v3e + v4e) / (2*L1_L2);

  // Estimate the distance and the turn angle
  ds = ve * dt;
  dns = vne * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta / 2) - dns * sin(thetae + dtheta / 2);
  ye += ds * sin(thetae + dtheta / 2) + dns * cos(thetae + dtheta / 2);
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

  robot.IRLine_Front.sense_cross = robot.IRLine_Front.SenseIRLineCrosses();
  robot.IRLine_Back.sense_cross = robot.IRLine_Back.SenseIRLineCrosses();
  robot.IRLine_Front.IR_total = 0;
  robot.IRLine_Back.IR_total = 0;
  for (int i = 0; i < 5; i++)
  {
    robot.IRLine_Front.IR_total += robot.IRLine_Front.IR_values[i];
  }
  for (int i = 0; i < 5; i++)
  {
    robot.IRLine_Back.IR_total += robot.IRLine_Back.IR_values[i];
  }

  // calc alfa
  // alfa = atan((robot.IRLine_Front.dist_center - robot.IRLine_Back.dist_center) / D12);

  // Line Crosses must be first to calculate variable "total" and "last_total"
  if (robot.IRLine_Front.sense_cross == true)
  {
    // Na linha de coordenad
    if (((abs(ye)) < TOL_DIST_LOC))
    {
      // CAUTION! difference between marks must be greatxer than TOL_DIST_LOC
      if (abs(xe - (0.225)) < TOL_DIST_LOC)
      {
        xe = 0.225;
        ye = 0;
      }
      if (abs(xe - (0.375)) < TOL_DIST_LOC)
      {
        xe = 0.375;
        ye = 0;
      }
      if (abs(xe - (0.735)) < TOL_DIST_LOC)
      {
        xe = 0.735;
        ye = 0;
      }
      if (abs(xe) < TOL_DIST_LOC)
      {
        xe = 0;
        ye = 0;
      }
    }

    if (((abs(xe)) < TOL_DIST_LOC))
    {
      // CAUTION! difference between marks must be greatxer than TOL_DIST_LOC
      if (abs(ye + (0.735)) < TOL_DIST_LOC)
      {
        xe = 0;
        ye = -0.735;
      }
      if (abs(ye + (1.030)) < TOL_DIST_LOC)
      {
        xe = 0;
        ye = -1.030;
      }
      if (abs(ye + (1.118)) < TOL_DIST_LOC)
      {
        xe = 0;
        ye = -1.118;
      }
      if (abs(ye + (1.330)) < TOL_DIST_LOC)
      {
        xe = 0;
        ye = -1.330;
      }
      if (abs(ye + (1.480)) < TOL_DIST_LOC)
      {
        xe = 0;
        ye = -1.480;
      }
    }

    if (((abs(xe - 0.735)) < TOL_DIST_LOC))
    {
      // CAUTION! difference between marks must be greatxer than TOL_DIST_LOC
      if (abs(ye + (0.155)) < TOL_DIST_LOC)
      {
        xe = 0.735;
        ye = -0.155;
      }
      if (abs(ye + (0.310)) < TOL_DIST_LOC)
      {
        xe = 0.735;
        ye = -0.310;
      }
      if (abs(ye + (0.465)) < TOL_DIST_LOC)
      {
        xe = 0.735;
        ye = -0.465;
      }
      if (abs(ye + (0.735)) < TOL_DIST_LOC)
      {
        xe = 0.735;
        ye = -0.735;
      }
      if (abs(ye + (1.480)) < TOL_DIST_LOC)
      {
        xe = 0.735;
        ye = -1.480;
      }
    }

    if (((abs(ye + 0.735)) < TOL_DIST_LOC))
    {
      // CAUTION! difference between marks must be greatxer than TOL_DIST_LOC
      if (abs(xe - (0.22)) < TOL_DIST_LOC)
      {
        xe = 0.22;
        ye = -0.735;
      }
      if (abs(xe - (0.375)) < TOL_DIST_LOC)
      {
        xe = 0.375;
        ye = -0.735;
      }
      if (abs(xe - (0.525)) < TOL_DIST_LOC)
      {
        xe = 0.525;
        ye = -0.735;
      }
    }

    if (((abs(ye + 1.480)) < TOL_DIST_LOC))
    {
      // CAUTION! difference between marks must be greatxer than TOL_DIST_LOC
      if (abs(xe - (0.525)) < TOL_DIST_LOC)
      {
        xe = 0.525;
        ye = -1.480;
      }
      if (abs(xe - (0.375)) < TOL_DIST_LOC)
      {
        xe = 0.375;
        ye = -1.480;
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
    v1ref = -(v + vn - w * L1_L2 / 2);
    v2ref = -(-v + vn - w * L1_L2 / 2);
    v3ref = -(v - vn - w * L1_L2 / 2);
    v4ref = -(-v - vn - w * L1_L2 / 2);

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
  alfa = atan2(yf - ye, xf - xe);

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
    v = 0;
    vn = 0;
    // at_target_pos = true;
  }
  else if (error_dist < DIST_DA)
  {
    v = (VEL_LIN_NOM / 3) * cos(ang_target - thetae);
    vn = (VEL_LIN_NOM / 3) * sin(ang_target - thetae);
  }
  else
  {
    v = VEL_LIN_NOM * cos(ang_target - thetae);
    vn = VEL_LIN_NOM * sin(ang_target - thetae);
  }
  if (abs(error_finalrot) < TOL_FINTHETA)
  {
    w = 0;
    // at_target_ang = true;
  }
  else if (abs(error_finalrot) < THETA_DA)
  {
    w = signal(error_finalrot) * VEL_ANG_NOM / 3;
  }
  else
  {
    w = signal(error_finalrot) * VEL_ANG_NOM;
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

void robot_t::Correct_Line(float kvn, float kA)
{

  robot.IRLine_Back.S = robot.IRLine_Back.Dist_Sensor();
  robot.IRLine_Front.S = robot.IRLine_Front.Dist_Sensor();
  alfa = atan((robot.IRLine_Front.S - robot.IRLine_Back.S) / D12);
  dist_ = (robot.IRLine_Back.S + robot.IRLine_Front.S) / 2;
  float total1 = 0;
  float total2 = 0;
  float x;
  for (int i = 0; i < 5; i++)
  {
    total1 += robot.IRLine_Front.IR_values[i];
  }
  for (int i = 0; i < 5; i++)
  {
    total2 += robot.IRLine_Back.IR_values[i];
  }
  /*
      if (total1 < 700 || total2 < 700)
      {
        ajustar_a = 2;
        ajustar_vn = 2;
        w = 0;
        vn = 0;
      }

      else if (abs(alfa) > 1.3)
      {
        ajustar_a = 1;

        if (abs(alfa) < 1.4)
        {
          w = -5 * kA * alfa;
        }

        else if (abs(alfa) < 1.6)
        {
          w = -20 * kA * alfa;
        }
        else
        {
          w = -70 * kA * alfa;
        }
      }

      else
      {
        ajustar_a = 0;
        w = 0;
      }
      if (abs(dist_) > 1)
      {
        // Ajustar Dist_to_center
        ajustar_vn = 1;
        if (abs(dist_) < 5)
        {

          vn = -kvn * dist_;
        }
        else if (abs(dist_) >= 5)
        {
          vn = 0;
        }
      }
      else
      {
        ajustar_vn = 0;
        vn = 0;
      }*/

  if (total1 < 700 || total2 < 700 || total1 > 2500 || total2 > 2500)
  {
    ajustar_a = 2;
    ajustar_vn = 2;
    w = 0;
    vn = 0;
  }

  else if (abs(dist_) > 0.8)
  {
    // Ajustar Dist_to_center
    ajustar_vn = 1;
    ajustar_a = 0;
    w = 0;

    if (abs(dist_) < 5)
    {

      vn = -kvn * dist_;
    }
    else if (abs(dist_) >= 5)
    {
      vn = 0;
    }
  }

  else if (abs(alfa) > 1.3)
  {
    ajustar_a = 1;
    ajustar_vn = 0;
    vn = 0;
    if (abs(alfa) < 1.4)
    {
      w = -5 * kA * alfa;
    }

    else if (abs(alfa) < 1.6)
    {
      w = -20 * kA * alfa;
    }
    else
    {
      w = -70 * kA * alfa;
    }
  }

  else
  {
    ajustar_a = 0;
    w = 0;
    ajustar_vn = 0;
    vn = 0;
  }
}

void robot_t::followLineLeft_ze(float v_frente)
{
}