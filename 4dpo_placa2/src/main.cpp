// 2023 Paulo Costa
// Periodic Interrupts for Quadrature Encoder reading and serial commands example
// It also has the PWM generation code to drive an H-bridge like the DRV8212PDSGR

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#define max_wifi_str 32

char ssid[max_wifi_str];
char password[max_wifi_str];

const char *fname_wifi = "/wifi.txt";

int udp_on, ip_on;

WiFiUDP Udp;
unsigned int localUdpPort = 4224; // local port to listen on

#define UDP_MAX_SIZE 512
uint8_t UdpInPacket[UDP_MAX_SIZE];  // buffer for incoming packets
uint8_t UdpOutPacket[UDP_MAX_SIZE]; // buffer for outgoing packets
int UdpBufferSize = UDP_MAX_SIZE;

#include "pico4drive.h"
//pico4drive_t pico4drive;

#include "PicoEncoder.h"

// Ajustar Encoders para os motores do braço e Roda
#define ENC1_PIN_A 2
#define ENC1_PIN_B 3

#define ENC2_PIN_A 4
#define ENC2_PIN_B 5

//#define ENC3_PIN_A 6
//#define ENC3_PIN_B 7

#define ENC4_PIN_A 8
#define ENC4_PIN_B 10

#define NUM_ENCODERS 4
PicoEncoder encoders[NUM_ENCODERS];
pin_size_t encoder_pins[NUM_ENCODERS] = {ENC1_PIN_A, ENC2_PIN_A};

bool calibration_requested;

#define TEST_PIN 27
// Ajustar Pins de alimentação dos motores do braço e Roda

#define MOTOR1A_PIN 11
#define MOTOR1B_PIN 10

#define MOTOR2A_PIN 13
#define MOTOR2B_PIN 12

#define SOLENOID_PIN_A 14
#define SOLENOID_PIN_B 15

#define MOTOR4A_PIN 17
#define MOTOR4B_PIN 16



int debug;

// #define HAS_VL53L0X 1
#include <Wire.h>
#ifdef HAS_VL53L0X
#include <VL53L0X.h>
VL53L0X tof;
#endif

// Não sei o que faz, não mexer
#ifdef HAS_INA266
#include <INA226_WE.h>

// There are several ways to create your INA226 object:
// INA226_WE ina226 = INA226_WE(); -> uses I2C Address = 0x40 / Wire
// INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
// INA226_WE ina226 = INA226_WE(&Wire); -> uses I2C_ADDRESS = 0x40, pass any Wire Object
// INA226_WE ina226 = INA226_WE(&Wire, I2C_ADDRESS);

INA226_WE ina226 = INA226_WE(0x40);
#endif

#include "arm.h"
#include "carrosel.h"

//void init_control(robot_t  &robot);
//void control(robot_t &robot);

void init_control(arm_t &arm, carrosel_t &carrosel);
void control(arm_t &arm, carrosel_t &carrosel);

//void init_control(carrosel_t  &carrosel);
//void control(carrosel_t &carrosel);

PID_pars_t arm_PID, carrosel_PID;

uint32_t interval, last_cycle;
uint32_t loop_micros;

// Read Sensors :


void readIRSensors(IRLine_t &IRLine)
{
  byte c; // Read the five IR sensors using the AD converter
  for (c = 0; c < IRSENSORS_COUNT; c++)
  {
    IRLine.IR_values[(IRSENSORS_COUNT - 1) - c] = 1023 - pico4drive.read_adc(3 + c);
  }
}

uint32_t encodeIRSensors(void)
{
  byte c;                                           // Encode five IR sensors with 6 bits for each sensor
  uint32_t result = carrosel.IR.IR_values[0] >> 4; // From 10 bits to 6 bits
  for (c = 1; c < IRSENSORS_COUNT; c++)
  {
    result = (result << 6) | (carrosel.IR.IR_values[c] >> 4);
  }
  return result;
}

// Não sei o que faz, não mexer

void set_interval(float new_interval)
{
  interval = new_interval * 1000000L; // In microseconds
  arm.dt = new_interval;            // In seconds
  arm_PID.dt = arm.dt;
  carrosel.dt = new_interval;
  carrosel_PID.dt = carrosel.dt;
}

// Remote commands-Evitar Mexer

#include "gchannels.h"
#include "file_gchannels.h"

gchannels_t udp_commands;
gchannels_t serial_commands;
gchannels_t serial1_commands;
commands_list_t pars_list;

const char *pars_fname = "pars.cfg";
bool load_pars_requested = false;

void process_command(command_frame_t frame)
{
  pars_list.process_read_command(frame);

  if (frame.command_is("moA"))
  { // The 'mo'de command ...
    arm.control_mode = (control_mode_arm)frame.value;
  }
  else if (frame.command_is("moC"))
  { // The 'mo'de command ...
    carrosel.control_mode = (control_mode_carrosel)frame.value;
  }
  else if (frame.command_is("u1A"))
  { // The 'u1' command sets the voltage for motor 1
    arm.u_req = frame.value;
  }
  else if (frame.command_is("u1C"))
  { // The 'u1' command sets the voltage for motor 1
    carrosel.u_req = frame.value;
  }
  else if (frame.command_is("w1A"))
  {
    arm.w_req = -frame.value;
  }
  else if (frame.command_is("sl"))
  {
    carrosel.solenoid_PWM = frame.value;
  }
  else if (frame.command_is("w1C"))
  {
    carrosel.w_req = -frame.value;
  }
  else if (frame.command_is("p1A"))
  {
    arm.p_req = (frame.value*TWO_PI)/360;
  }
  else if (frame.command_is("p1C"))
  {
    carrosel.p_req = (frame.value*TWO_PI)/360;
  }
  else if (frame.command_is("stA"))
  {
    arm.pfsm->set_new_state(frame.value);
    arm.pfsm->update_state();
  }
  else if (frame.command_is("stC"))
  {
    carrosel.pfsm->set_new_state(frame.value);
    carrosel.pfsm->update_state();
  }
  else if (frame.command_is("dt"))
  {
    set_interval(frame.value);
  }
  else if (frame.command_is("vA"))
  {
    arm.v_req = frame.value;
  }
  else if (frame.command_is("vC"))
  {
    carrosel.v_req = frame.value;
  }
  else if (frame.command_is("wA"))
  {
    arm.w_req = frame.value;
  }
  else if (frame.command_is("wC"))
  {
    carrosel.w_req = frame.value;
  }
  else if (frame.command_is("sl"))
  {
    arm.solenoid_PWM = frame.value;
  }
  else if (frame.command_is("pl"))
  {
    // load_commands(pars_fname, serial_commands);
    load_pars_requested = true;
  }
  else if (frame.command_is("PICK_OK"))
  {
    carrosel.pick_ok = frame.value;
  }
  else if (frame.command_is("DROP_OK"))
  {
    carrosel.drop_ok = frame.value;
  }
  else if (frame.command_is("stC"))
  {
    carrosel.drop_ok = 0;
  }
  else if (frame.command_is("ps"))
  {
    save_commands(pars_fname, pars_list, serial_commands);
  }
  else if (frame.command_is("ssid"))
  {
    strncpy(ssid, frame.text, max_wifi_str - 1);
    ssid[max_wifi_str - 1] = 0;
  }
  else if (frame.command_is("pass"))
  {
    if (strlen(frame.text) < 8)
      return;
    strncpy(password, frame.text, max_wifi_str - 1);
    password[max_wifi_str - 1] = 0;
  }
  else if (frame.command_is("wifi"))
  {
    if (frame.value == 1)
    {
      if (WiFi.connected())
        WiFi.end();
      WiFi.begin(ssid, password);
    }
    else if (frame.value == 0)
    {
      WiFi.end();
    }
  }
  else if (frame.command_is("cal"))
  {
    calibration_requested = true;
    // set_interval(frame.value);

  } // Put here more commands...
}


// void init_PIO_dual_encoders(int enc1_pin_A, int enc2_pin_A);
// int read_PIO_encoder(int sm);

void read_PIO_encoders(void)
{
  encoders[0].update();
  encoders[1].update();
  arm.enc = encoders[1].speed;
  carrosel.enc = encoders[0].speed;
}

void serial_write(const char *buffer, size_t size)
{
  Serial.write(buffer, size);
  if (udp_on)
  {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(buffer, size);
    // Serial.print("Sent="); Serial.println(Udp.endPacket());
    Udp.endPacket();
  }
}

void serial1_write(const char *buffer, size_t size)
{
  Serial1.write(buffer, size);
}

const char *encToString(uint8_t enc)
{
  switch (enc)
  {
  case ENC_TYPE_NONE:
    return "NONE";
  case ENC_TYPE_TKIP:
    return "WPA";
  case ENC_TYPE_CCMP:
    return "WPA2";
  case ENC_TYPE_AUTO:
    return "AUTO";
  }
  return "UNKN";
}

void wifi_list(void)
{
  Serial.printf("Beginning scan at %d\n", millis());
  int cnt = WiFi.scanNetworks();
  if (!cnt)
  {
    Serial.printf("No networks found\n");
  }
  else
  {
    Serial.printf("Found %d networks\n\n", cnt);
    Serial.printf("%32s %5s %2s %4s\n", "SSID", "ENC", "CH", "RSSI");
    for (int i = 0; i < cnt; i++)
    {
      uint8_t bssid[6];
      WiFi.BSSID(i, bssid);
      Serial.printf("%32s %5s %2d %4d\n", WiFi.SSID(i), encToString(WiFi.encryptionType(i)), WiFi.channel(i), WiFi.RSSI(i));
    }
  }
}




/*

---------------------------------------SETUP ---------------------------------------

*/

void setup()
{
  // Set the pins as input or output as needed
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ENC1_PIN_A, INPUT_PULLUP);
  pinMode(ENC1_PIN_B, INPUT_PULLUP);
  pinMode(ENC2_PIN_A, INPUT_PULLUP);
  pinMode(ENC2_PIN_B, INPUT_PULLUP);
  //pinMode(ENC3_PIN_A, INPUT_PULLUP);
  //pinMode(ENC3_PIN_B, INPUT_PULLUP);
  pinMode(ENC4_PIN_A, INPUT_PULLUP);
  pinMode(ENC4_PIN_B, INPUT_PULLUP);

  pinMode(switchPIN, INPUT_PULLUP);

  pinMode(TEST_PIN, OUTPUT);

  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);

  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);

  pinMode(SOLENOID_PIN_A, OUTPUT);
  pinMode(SOLENOID_PIN_B, OUTPUT);

//  Serial2.setRX(IR_RXPin);
//  Serial2.setTX(IR_TXPin);
  

  // Encoders Setup
  encoders[0].begin(encoder_pins[0]);
  encoders[1].begin(encoder_pins[1]);
  //encoders[2].begin(encoder_pins[2]);
  //encoders[3].begin(encoder_pins[3]);

  pico4drive.init();
  
  analogReadResolution(10);
  // Register Commands
  pars_list.register_command("kfA", &(arm_PID.Kf));
  pars_list.register_command("kfC", &(carrosel_PID.Kf));
  pars_list.register_command("kcA", &(arm_PID.Kc));
  pars_list.register_command("kcC", &(carrosel_PID.Kc));
  pars_list.register_command("kiA", &(arm_PID.Ki));
  pars_list.register_command("kiC", &(carrosel_PID.Ki));
  pars_list.register_command("kdA", &(arm_PID.Kd));
  pars_list.register_command("kdC", &(carrosel_PID.Kd));
  pars_list.register_command("kfdA", &(arm_PID.Kfd));
  pars_list.register_command("kfdC", &(carrosel_PID.Kfd));
  pars_list.register_command("dzA", &(arm_PID.dead_zone));
  pars_list.register_command("dzC", &(carrosel_PID.dead_zone));
  pars_list.register_command("kfpA", &(arm_PID.Kf_p));
  pars_list.register_command("kfpC", &(carrosel_PID.Kf_p));
  pars_list.register_command("kcpA", &(arm_PID.Kc_p));
  pars_list.register_command("kcpC", &(carrosel_PID.Kc_p));
  pars_list.register_command("kipA", &(arm_PID.Ki_p));
  pars_list.register_command("kipC", &(carrosel_PID.Ki_p));
  pars_list.register_command("kdpA", &(arm_PID.Kd_p));
  pars_list.register_command("kdpC", &(carrosel_PID.Kd_p));
  pars_list.register_command("kfdpA", &(arm_PID.Kfd_p));
  pars_list.register_command("kfdpC", &(carrosel_PID.Kfd_p));







  // pars_list.register_command("fk", &(arm.i_lambda));
  // pars_list.register_command("ssid", ssid, max_wifi_str);
  // pars_list.register_command("pass", password, max_wifi_str);

  udp_commands.init(process_command, serial_write);

  serial_commands.init(process_command, serial_write);

  serial1_commands.init(process_command, serial1_write);

  arm.pchannels = &serial_commands;
  //robot.pchannels = &serial_commands;
  carrosel.pchannels = &serial1_commands;

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(2400, SERIAL_8N1);


  float control_interval = 0.04; // In seconds

  // PID INITIAL PARAMETERS

  arm_PID.Kf = 0.3;
  arm_PID.Kc = 2.15;
  arm_PID.Ki = 3.3;
  arm_PID.Kd = 0;
  arm_PID.Kfd = 0;
  arm_PID.Kc_p = 15;
  arm_PID.Kd_p = 0.3;
  arm_PID.dt = 0.04;
  arm_PID.dead_zone = 0;

  carrosel_PID.Kf = 0.3;
  carrosel_PID.Kc = 2.5;
  carrosel_PID.Ki = 4.5;
  carrosel_PID.Kd = 0;
  carrosel_PID.Kfd = 0;
  carrosel_PID.Kc_p = 30;
  carrosel_PID.Kd_p = 0.3;
  carrosel_PID.dt = 0.04;
  carrosel_PID.dead_zone = 0;

  arm.PID.init_pars(&arm_PID);
  carrosel.PID.init_pars(&carrosel_PID);
  
  // Outras Coisas
  strcpy(ssid, "5DPO-NETWORK");
  strcpy(password, "5dpo5dpo");

  load_commands(pars_fname, serial_commands);

  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);

  // Start WiFi with supplied parameters
  WiFi.begin(ssid, password);

  // if (ITimer1.attachInterrupt(40000, timer_handler))
  //   Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  // else
  //   Serial.println("Can't set ITimer. Select another freq. or timer");

  // init_OTA();

  // Wire.setSDA(8);
  // Wire.setSCL(9);

  // Wire.begin();

#ifdef HAS_INA266

  while (!ina226.init())
  {
    Serial.println("could not connect ina226!");
    delay(100);
  }

  setup_ina226();
#endif

#ifdef HAS_VL53L0X

  // tof.setAddress(0x22);

  tof.setTimeout(100);
  while (!tof.init())
  {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }

  // Reduce timing budget to 20 ms (default is about 33 ms)
  // tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startContinuous(0);

#endif

  
  set_interval(control_interval); // In seconds
  init_control(arm, carrosel);


  arm.pos_init();
  carrosel.carrosel_pos_init();

  //init_control(carrosel);
}
/*



------------------LOOP CODE-----------------------
*/
char irrecvdata;
int irrecvbuffer_index = -1;
char irrecvbuffer[5];
bool ReceivedInstruction = false;
void loop()
{
  if (Serial2.available() ) {
    irrecvdata = Serial2.read();
    //Serial.write(irrecvdata); 
    if(irrecvdata == 'U') {
      Serial.print("!");
    } else if(irrecvdata == 'W') {
      irrecvbuffer_index = 0;
    } else if(irrecvbuffer_index >= 0) {
      irrecvbuffer[irrecvbuffer_index] = irrecvdata;
      if(irrecvbuffer[irrecvbuffer_index] == 'w' || irrecvbuffer[irrecvbuffer_index] == 'o' || irrecvbuffer[irrecvbuffer_index] == 'u'){
        irrecvbuffer_index++;
        if(irrecvbuffer_index >= 4) {
          irrecvbuffer[irrecvbuffer_index] = '\0';
          irrecvbuffer_index = -1;
          Serial.println("Received Instruction!");
          Serial.println(irrecvbuffer);
          //serial_commands.send_command("Box", irrecvbuffer);
          ReceivedInstruction = true;
          //control(arm, carrosel);
          //robot.state = SETUP;
          
        }
      } else {
        Serial.write(irrecvdata);
        Serial.println();
      }
    }
  }
  if (WiFi.connected() && !ip_on)
  {
    // Connection established
    serial_commands.send_command("msg", (String("Pico W is connected to WiFi network with SSID ") + WiFi.SSID()).c_str());

    // Print IP Address
    ip_on = Udp.begin(localUdpPort);
    Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  }


  if (ip_on)
  {
    // ArduinoOTA.handle();

    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      int i;
      udp_on = 1;
      // receive incoming UDP packets

      // Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0)
      {
        UdpInPacket[len] = 0;
      }
      // Serial.printf("UDP packet contents (as string): %s\n", UdpInPacket);

      for (i = 0; i < len; i++)
      {
        udp_commands.process_char(UdpInPacket[i]);
        // Serial.write(UdpInPacket[i]);
      }
    }
  }

  uint8_t b;
  if (Serial.available())
  { // Only do this if there is serial data to be read

    b = Serial.read();
    serial_commands.process_char(b);
    // Serial.write(b);
  }
  uint8_t x;

  if (Serial1.available())
  { // Only do this if there is serial data to be read

    x = Serial1.read();
    serial1_commands.process_char(x);
    // Serial.write(b);
  }

  pico4drive.update();

  if (load_pars_requested)
  {
    load_commands(pars_fname, serial_commands);
    load_pars_requested = false;
  }

  // Do this only every "interval" microseconds
  uint32_t now = micros();
  uint32_t delta = now - last_cycle;
  if (delta >= interval) {
    loop_micros = micros();
    last_cycle = now;
    // last_cycle += interval;


#ifdef HAS_VL53L0X
  if (tof.readRangeAvailable())
  {
    arm.prev_tof_dist = arm.tof_dist;
    arm.tof_dist = tof.readRangeMillimeters() * 1e-3;
  }
#endif

#ifdef HAS_INA266
  ina226.readAndClearFlags();
  float shuntVoltage_mV = ina226.getShuntVoltage_mV();
  float busVoltage_V = ina226.getBusVoltage_V();
  float current_mA = -ina226.getCurrent_mA();
  // float power_mW = ina226.getBusPower();
  // float loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);

  arm.i_sense = arm.i_lambda * arm.i_sense + (1 - arm.i_lambda) * current_mA * 1e-3;
  arm.u_sense = busVoltage_V * 1000;
#endif

  //----------- Código Para Roda e Braço -------------------------------------------------------
  // Read and process sensors


    read_PIO_encoders();

   
    arm.odometry();
    carrosel.odometry();

    control(arm, carrosel);

    // Calc outputs
    // arm.accelerationLimit();
    

    arm.calcMotorsVoltage();
    carrosel.calcMotorsVoltage();

    arm.PWM = pico4drive.voltage_to_PWM(arm.u);
    pico4drive.set_driver_PWM(arm.PWM, MOTOR2A_PIN, MOTOR2B_PIN);

    carrosel.PWM = pico4drive.voltage_to_PWM(carrosel.u);
    pico4drive.set_driver_PWM(carrosel.PWM, MOTOR1A_PIN, MOTOR1B_PIN);

    pico4drive.set_driver_PWM(carrosel.solenoid_PWM, SOLENOID_PIN_A, SOLENOID_PIN_B);

    readIRSensors(carrosel.IR);

    //control(arm, carrosel);
    
    
  
  
  //----------- Fim Do Código da Roda e do Braço------------------------------------------------
     

      // Debug information
      serial1_commands.send_command("Box0", irrecvbuffer);
      
      serial_commands.send_command("swPin", digitalRead(switchPIN));
      serial_commands.send_command("dte", delta);

      serial_commands.send_command("u1A", arm.u);
      serial_commands.send_command("u1C", carrosel.u);

      serial_commands.send_command("encA", arm.enc);
      serial_commands.send_command("encC", carrosel.enc);
      
      serial_commands.send_command("Vbat", pico4drive.battery_voltage);

      serial_commands.send_command("veA", arm.ve);
      serial_commands.send_command("weA", arm.we);

      serial_commands.send_command("veC", carrosel.ve);
      serial_commands.send_command("weC", carrosel.we);

      //serial_commands.send_command("w1", arm.we);

      serial_commands.send_command("posA", ((arm.p_e*360)/TWO_PI));
      serial_commands.send_command("posC", (carrosel.p_e*360)/TWO_PI);

      serial_commands.send_command("sl", carrosel.solenoid_PWM);

      serial_commands.send_command("modeA", arm.control_mode);
      serial_commands.send_command("modeC", carrosel.control_mode);

      serial_commands.send_command("kcA", arm_PID.Kc);
      serial_commands.send_command("kiA", arm_PID.Ki);
      serial_commands.send_command("kdA", arm_PID.Kd);
      serial_commands.send_command("kfA", arm_PID.Kf);

      serial_commands.send_command("kcpA", arm_PID.Kc_p);
      serial_commands.send_command("kipA", arm_PID.Ki_p);
      serial_commands.send_command("kdpA", arm_PID.Kd_p);
      serial_commands.send_command("kfpA", arm_PID.Kf_p);

      serial_commands.send_command("kcC", carrosel_PID.Kc);
      serial_commands.send_command("kiC", carrosel_PID.Ki);
      serial_commands.send_command("kdC", carrosel_PID.Kd);
      serial_commands.send_command("kfC", carrosel_PID.Kf);

      serial_commands.send_command("kcpC", carrosel_PID.Kc_p);
      serial_commands.send_command("kipC", carrosel_PID.Ki_p);
      serial_commands.send_command("kdpC", carrosel_PID.Kd_p);
      serial_commands.send_command("kfpC", carrosel_PID.Kf_p);

      serial_commands.send_command("IRB0", carrosel.IR.IR_values[0]);
      serial_commands.send_command("IRB1", carrosel.IR.IR_values[1]);
      serial_commands.send_command("IRB2", carrosel.IR.IR_values[2]);
      serial_commands.send_command("IRB3", carrosel.IR.IR_values[3]);
      serial_commands.send_command("IRB4", carrosel.IR.IR_values[4]);
      serial_commands.send_command("IRB5", carrosel.IR.IR_values[5]);

      serial_commands.send_command("stA", arm.pfsm->state);
      serial_commands.send_command("stC", carrosel.pfsm->state);
      serial_commands.send_command("caixa", carrosel.counter);

      serial_commands.send_command("IP", WiFi.localIP().toString().c_str());

      

      // serial_commands.send_command("d0", arm.tof_dist);

      // serial_commands.send_command("pr", arm.IR.pos_right);
      // serial_commands.send_command("pl", arm.IR.pos_left);

      serial_commands.send_command("mA", arm.PWM);
      serial_commands.send_command("mC", carrosel.PWM);
    

      pars_list.send_sparse_commands(serial_commands);

      Serial.print(" cmd: ");
      Serial.print(serial_commands.frame.command);
      Serial.print("; ");

      debug = serial_commands.out_count;
      serial_commands.send_command("dbg", 5);
      serial_commands.send_command("loop", micros() - loop_micros);

      serial_commands.flush();
      Serial.println();

      serial1_commands.send_command("IRB10", carrosel.IR.IR_values[0]);
      serial1_commands.send_command("IRB11", carrosel.IR.IR_values[1]);
      serial1_commands.send_command("IRB12", carrosel.IR.IR_values[2]);
      serial1_commands.send_command("IRB13", carrosel.IR.IR_values[3]);
      serial1_commands.send_command("IRB14", carrosel.IR.IR_values[4]);
      serial1_commands.send_command("IRB15", carrosel.IR.IR_values[5]);
      serial1_commands.send_command("a", 1);


      serial1_commands.send_command("PICK_DONE", carrosel.pick_done);
      serial1_commands.send_command("DROP_DONE", carrosel.drop_done);

      serial1_commands.flush();
  }

}
