#include "PID.h"
#include "IRLine.h"
#include "Arduino.h"
#include "state_machines.h"
#include "gchannels.h"

#define switchPIN 9

typedef enum
{
    cm_pos,
    cm_voltage
} control_mode_arm;

class arm_t
{
public:
 // velocidades estimadas
 float ve, we;
 // impulsos contados
 int enc;
 // tensão motor
 float u, u_req;
 // Velocidades pedidas
 float v_req, w_req;
 float v1ref, w1ref;
 float p_ref, p_req; //posição de referencia e pedida
 float p_e; //erro de posição
 float dt;
 
 control_mode_arm control_mode;
 IRLine_t IR;

 PID_t PID[0];
 gchannels_t *pchannels;

 arm_t();

 void odometry(void);
 void pos_init(void); //rotina que obrigue o braço a definir um ponto de referencia
 void pos_update(void);
 void set_pos(float pos);
 void calculate_motors_voltage(void);

 void send_command(const char *command, float par);
 void send_command(const char *command, const char *par);
};

extern arm_t arm;