/* Copyright (c) 2023  Paulo Costa
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

#ifndef ROBOT_H
#define ROBOT_H
#endif

#include <Arduino.h>
#include <math.h>
#include "PID.h"
#include "IRLine.h"
#include "state_machines.h"
#include "gchannels.h"

#ifndef NUM_WHEELS
#define NUM_WHEELS 4

typedef enum
{
  cm_voltage,
  cm_pid,
  cm_kinematics,
  cm_pos,
  cm_IR
} control_mode_t;

class robot_t
{
public:
  int enc1, enc2, enc3, enc4;
  int Senc1, Senc2;
  float w1e, w2e, w3e, w4e;
  float v1e, v2e, v3e, v4e;
  float p1e;
  float alfa, dist_;
  float ve, we;
  float ds, dtheta;
  float rel_s, rel_theta, rel_ns;
  float xe, ye, thetae;
  float dns;

  float D12, D2z;

  float dt;
  float v, vn, w;
  float v_req, vn_req, w_req;
  float dv_max, dw_max;

  float wheel_radius, wheel_dist, L1_L2;

  float v1ref, v2ref, v3ref, v4ref;
  float w1ref, w2ref, w3ref, w4ref;
  float p1ref;
  float u1, u2, u3, u4;
  float u1_req, u2_req, u3_req, u4_req;
  float i_sense, u_sense;
  float i_lambda;
  int PWM_1, PWM_2, PWM_3, PWM_4;
  float vne;
  bool stoped;
  // int PWM_1_req, PWM_2_req;
  float w1_req, w2_req, w3_req, w4_req;
  float p1_req;
  control_mode_t control_mode;
  float follow_v, follow_k;
  float ajustar_a, ajustar_vn;
  int Box_picked, etapa;
  bool pick_ok, pick_done, drop_ok, drop_done;
  float time;

  IRLine_t IRLine_Front,
      IRLine_Back, last_IRLine_Front, last_IRLine_Back;
  // IRLine_t IRLine;
  PID_t PID[NUM_WHEELS];

  int solenoid_PWM;
  int led;

  // IRLine_t IRLine;
  float tof_dist, prev_tof_dist;

  int LastTouchSwitch, TouchSwitch;
  state_machine_t *pfsm;
  gchannels_t *pchannels;

  bool control_event;
  bool new_line_read = false;
  bool at_destin = false;

  robot_t();

  void odometry(void);
  void setRobotVW(float Vnom, float VNnom, float Wnom);
  bool line_marker_front(void);
  void localization(void);

  void accelerationLimit(void);
  void calcMotorsVoltage(void);

  float dist(float xd, float yd);
  void dist2Line(float xe, float ye, float xi, float yi, float xf, float yf, float &xr, float &yr);
  void gotoXYTheta(float xf, float yf, float thf);
  void FollowLine(float xi, float yi, float xf, float yf, float thf);

  void followLineRight(float Vnom, float K);
  void followLineLeft(float Vnom, float K);

  void followLineRight_ze(float Vnom, float K);
  void followLineLeft_ze(float Vnom);

  void Correct_Line(float kVN, float kA);
  void Follow_Line_ze(float Vnom, float K);

  void send_command(const char *command, float par);
  void send_command(const char *command, const char *par);
};

extern robot_t robot;

#endif // ROBOT_H
