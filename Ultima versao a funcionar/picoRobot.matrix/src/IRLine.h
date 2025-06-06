/* Copyright (c) 2019  Paulo Costa
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

#ifndef IRLINE_H
#define IRLINE_H

#include "Arduino.h"

#define IRSENSORS_COUNT 5

class IRLine_t
{
  public:
    float pos_left, pos_right, dist_center;
    int IR_values[IRSENSORS_COUNT];
    int last_IR_values[IRSENSORS_COUNT];
    int IR_tot_tresh;
    int IR_WaterLevel;
    int IR_tresh, IR_max;

    int crosses;
    int cross_count, last_cross_count;
    int cross_tresh;
    int black_cross_level;
    int cross_total_tresh;

    int total, last_total;

    float blacks;

    IRLine_t();
    
    void calibrate(void);
    
    bool calcIRLineEdgeLeft(void);
    bool calcIRLineEdgeRight(void);
    bool SenseIRLineCrosses(void);
};

#endif // IRLINE_H
