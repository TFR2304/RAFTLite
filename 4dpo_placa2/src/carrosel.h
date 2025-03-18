#include "PID.h"

class carrosel_t
{
public:
 // velocidades estimadas
 float ve, we;
 // impulsos contados
 int enc1;
 // tensão motor
 float u1;
 // Velocidades pedidas
 float v_req, w_req;
 // posição estimada e pedida (em graus)
 float pos_e ;
 // Índice da posição pedida (1-4)
 float pos_req;

     IRLine_t IR;

 float pos[4];

 PID_t PID[1];

 carrosel_t();

 void odometry(void);
 void pos_update(void);
 bool set_pos(float pos);
 void set_w(float w);
 void calculate_motors_voltage(void);
};

extern carrosel_t carrosel;