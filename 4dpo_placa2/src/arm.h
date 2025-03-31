#include "PID.h"
#include "IRLine.h"
#include "Arduino.h"
#include "state_machines.h"
#include "gchannels.h"
#include <math.h>

#define switchPIN 9

typedef enum
{
    arm_cm_pos,
    arm_cm_voltage
} control_mode_arm;

class arm_t
{
public:
 // velocidades estimadas
 float ve, we;
 // impulsos contados
 int enc;
 //int counter;
 // tensão motor
 float u, u_req;
 float PWM;
 // Velocidades pedidas
 float v_req, w_req;
 float v1ref, w1ref;
 float p, p_ref, p_req; //posição de referencia e pedida
 float p_e; //posição estimada
 float dt;

 float solenoid_PWM;
 bool stoped;
 state_machine_t *pfsm;
 control_mode_arm control_mode;
 IRLine_t IR;

 PID_t PID;
 gchannels_t *pchannels;

 arm_t();

 void odometry(void);
 void pos_init(void); //rotina que obrigue o braço a definir um ponto de referencia
 void pos_update(void);
 void set_pos(float pos);
 void calcMotorsVoltage(void);

 void send_command(const char *command, float par);
 void send_command(const char *command, const char *par);
};

extern arm_t arm;