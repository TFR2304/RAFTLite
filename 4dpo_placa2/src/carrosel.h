#include "PID.h"
#include "IRLine.h"
#include "Arduino.h"
#include "gchannels.h"
#include "state_machines.h"

typedef enum
{
    carrosel_cm_pos,
    carrosel_cm_voltage
} control_mode_carrosel;

class carrosel_t
{
public:
 // velocidades estimadas
 float ve, we;
 // impulsos contados
 int enc;
 // tensão motor
 float u, u_req;
 float PWM;
 // Velocidades pedidas
 float v_req, w_req;
 float v1ref, w1ref;
 float p, p_ref, p_req; //posição de referencia e pedida
 float p_e; //posição estimada
 float dt;

 int idx_pos[4];

 float solenoid_PWM;
 bool stoped;
 state_machine_t *pfsm;
 control_mode_carrosel control_mode;
 IRLine_t IR;

 PID_t PID;
 gchannels_t *pchannels;
 carrosel_t();

 void odometry(void);
 void pos_update(void);
 bool set_pos(float pos);
 void set_w(float w);
 void carrosel_pos_init(void);
 void calcMotorsVoltage(void);

 void send_command(const char *command, float par);
 void send_command(const char *command, const char *par);
};

extern carrosel_t carrosel;