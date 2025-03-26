#include "PID.h"
#include "IRLine.h"
#include "Arduino.h"
#include "gchannels.h"
#include "state_machines.h"

typedef enum
{
    carrosel_cmc_pos,
    carrosel_cm_voltage
} control_mode_carrosel;

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
 gchannels_t *pchannels;
 state_machine_t *pfsm;
 control_mode_carrosel control_mode;

 carrosel_t();

 void odometry(void);
 void pos_update(void);
 bool set_pos(float pos);
 void set_w(float w);
 void calculate_motors_voltage(void);

 void send_command(const char *command, float par);
 void send_command(const char *command, const char *par);
};

extern carrosel_t carrosel;