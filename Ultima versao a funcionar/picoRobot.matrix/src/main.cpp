// 2024 Paulo Costa
// PicoRobot Hardware in the loop


#include <Arduino.h>

#include <WiFi.h>

//#include <RPi_Pico_TimerInterrupt.h>
//#include <ArduinoOTA.h>

#include <WiFiUdp.h>
#include <LittleFS.h>
#include "http_ota.h"

#define max_wifi_str 32

char ssid[max_wifi_str];
char password[max_wifi_str];

const char* fname_wifi = "/wifi.txt";

int udp_on, ip_on;

WiFiUDP Udp;
unsigned int localUdpPort = 4224;  // local port to listen on

#define UDP_MAX_SIZE 512
uint8_t UdpInPacket[UDP_MAX_SIZE];  // buffer for incoming packets
uint8_t UdpOutPacket[UDP_MAX_SIZE];  // buffer for outgoing packets
int UdpBufferSize = UDP_MAX_SIZE;

// Select the timer you're using, from ITimer0(0)-ITimer3(3)
// Init RPI_PICO_Timer
//RPI_PICO_Timer ITimer1(1);


#define TEST_PIN 27

// PWM stuff


int debug;
int analogWriteBits = 10; 
int analogWriteMax = (1 << analogWriteBits) - 1; 

int last_cycle_count;
int cycle_count;

bool debug_mode = true;

#include "robot.h"

void init_control(robot_t& robot);
void control(robot_t& robot);

#include "trajectories.h"

PID_pars_t wheel_PID_pars;


uint32_t interval, last_cycle;
uint32_t loop_micros;

void set_interval(float new_interval)
{
  interval = new_interval * 1000000L;   // In microseconds
  robot.dt = new_interval;   // In seconds
  wheel_PID_pars.dt = robot.dt;  
}

// Remote commands

#include "gchannels.h"
#include "file_gchannels.h"

gchannels_t udp_commands;
gchannels_t serial_commands;
commands_list_t pars_list;

const char* pars_fname = "pars.cfg";
bool load_pars_requested = false;

void process_command(command_frame_t frame)
{
  pars_list.process_read_command(frame);

  if (frame.command_is("go")) { // The 'go' command ...
    robot.control_event = true;
  
  } else if (frame.command_is("e1")) { // Encoder 1
    robot.enc1 = frame.value;

  } else if (frame.command_is("e2")) { // Encoder 2
    robot.enc2 = frame.value;

  } else if (frame.command_is("e3")) { // Encoder 1
    robot.enc3 = frame.value;

  } else if (frame.command_is("e4")) { // Encoder 2
    robot.enc4 = frame.value;
    
  } else if (frame.command_prefix_is("L")) { // Line sensors
    int i = frame.index;
    if (i >= 0 && i < IRSENSORS_COUNT) {
      robot.IRLine_Front.last_IR_values[i] = robot.IRLine_Front.IR_values[i]; 
      robot.IRLine_Front.IR_values[i] = frame.value * 10; // Scale it to the range of 0 - 1000
    }
    if (i >= IRSENSORS_COUNT && i < 10) {
      robot.IRLine_Back.last_IR_values[i - IRSENSORS_COUNT] = robot.IRLine_Back.IR_values[i - IRSENSORS_COUNT]; 
      robot.IRLine_Back.IR_values[i  - IRSENSORS_COUNT] = frame.value * 10; // Scale it to the range of 0 - 1000
    }
    if (i = 9) { robot.new_line_read = true; }
  
  } else if (frame.command_is("di")) { // Touch sensor
    robot.LastTouchSwitch = robot.TouchSwitch;
    robot.TouchSwitch = frame.value;

  } else if (frame.command_is("st")) { 
     robot.pfsm->set_new_state(frame.value);
     robot.pfsm->update_state();

  } else if (frame.command_is("mo")) { // The mode command
    robot.control_mode = (control_mode_t) frame.value;

  } else if (frame.command_is("u1")) { // The 'u1' command sets the voltage for motor 1
    robot.u1_req = frame.value;

  } else if (frame.command_is("u2")) { // The 'u2' command sets the voltage for motor 1
    robot.u2_req = frame.value;

  } else if (frame.command_is("dt")) { 
     set_interval(frame.value);

  } else if (frame.command_is("sl")) { 
    robot.solenoid_PWM = frame.value;    

  } else if (frame.command_is("xr")) { 
    robot.xe = frame.value;    

  } else if (frame.command_is("yr")) { 
    robot.ye = frame.value;    

  } else if (frame.command_is("tr")) { 
    robot.thetae = frame.value;    
    robot.max_w1e = 0;
    robot.max_w2e = 0;
    robot.max_w3e = 0;
    robot.max_w4e = 0;

  } else if (frame.command_is("xt")) { 
    traj.xt = frame.value;    

  } else if (frame.command_is("yt")) { 
    traj.yt = frame.value;    

  } else if (frame.command_is("pl")) { 
    //load_commands(pars_fname, serial_commands);
    load_pars_requested = true;

  } else if (frame.command_is("ps")) {  
    save_commands(pars_fname, pars_list, serial_commands);

  } else if (frame.command_is("ac")) {  
    last_cycle_count = cycle_count;
    cycle_count = frame.value; 

  } else if (frame.command_is("ssid")) { 
    strncpy(ssid, frame.text, max_wifi_str - 1);
    ssid[max_wifi_str - 1] = 0;
  
  } else if (frame.command_is("pass")) { 
    if (strlen(frame.text) < 8) return;
    strncpy(password, frame.text, max_wifi_str - 1);
    password[max_wifi_str - 1] = 0;    
    
  } else if (frame.command_is("wifi")) { 
    if (frame.value == 1) {
      if (WiFi.connected()) WiFi.end();
      WiFi.begin(ssid, password); 
    } else if (frame.value == 0) {
      WiFi.end();
    }

  } else if (frame.command_is("httpota")) { 
    if (WiFi.connected()) {
      http_ota.host = frame.text;
      http_ota.uri = "/picoRobot/firmware.bin"; 
      robot.stoped = true;
      http_ota.requested = true;
    }
  
  } // Put here more commands...
}


void send_file(const char* filename, int log_high)
{
  File f;
  f = LittleFS.open(filename, "r");
  if (!f) {
    serial_commands.send_command("err", filename);
    return;
  }

  serial_commands.flush();
  Serial.flush();

  int c;
  byte b, mask;
  if (log_high) mask = 0x80;
  else mask = 0;

  while(1) {
    c = f.read(&b, 1);
    if (c != 1) break;
    serial_commands.send_char(b | mask);
  }
  f.close();

  serial_commands.flush();
  Serial.flush();  
}

void serial_write(const char *buffer, size_t size)
{
  Serial.write(buffer, size);
  if (udp_on) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(buffer, size);
    //Serial.print("Sent="); Serial.println(Udp.endPacket());
    Udp.endPacket();  
  } 
}

const char *encToString(uint8_t enc) {
  switch (enc) {
    case ENC_TYPE_NONE: return "NONE";
    case ENC_TYPE_TKIP: return "WPA";
    case ENC_TYPE_CCMP: return "WPA2";
    case ENC_TYPE_AUTO: return "AUTO";
  }
  return "UNKN";
}


void wifi_list(void)
{
  Serial.printf("Beginning scan at %d\n", millis());
  int cnt = WiFi.scanNetworks();
  if (!cnt) {
    Serial.printf("No networks found\n");
  } else {
    Serial.printf("Found %d networks\n\n", cnt);
    Serial.printf("%32s %5s %2s %4s\n", "SSID", "ENC", "CH", "RSSI");
    for (int i = 0; i < cnt; i++) {
      uint8_t bssid[6];
      WiFi.BSSID(i, bssid);
      Serial.printf("%32s %5s %2d %4d\n", WiFi.SSID(i), encToString(WiFi.encryptionType(i)), WiFi.channel(i), WiFi.RSSI(i));
    }
  }
}

/*
void init_OTA(void)
{
  ArduinoOTA.setPort(2040); // this is default for RP 2040
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { 
    Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    } 
  });  
  
  ArduinoOTA.begin();    
}*/


void setup() 
{
  // Set the pins as input or output as needed
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(TEST_PIN, OUTPUT);

  analogReadResolution(10);

  analogWriteResolution(analogWriteBits);
  analogWriteFreq(16000); //16000 

  pars_list.register_command("kf", &(wheel_PID_pars.Kf));
  pars_list.register_command("kc", &(wheel_PID_pars.Kc));
  pars_list.register_command("ki", &(wheel_PID_pars.Ki));
  pars_list.register_command("kd", &(wheel_PID_pars.Kd));
  pars_list.register_command("kfd", &(wheel_PID_pars.Kfd));
  pars_list.register_command("dz", &(wheel_PID_pars.dead_zone));

  pars_list.register_command("at", &(traj.thetat));
  pars_list.register_command("xt", &(traj.xt));
  pars_list.register_command("yt", &(traj.yt));

  pars_list.register_command("xi", &(traj.xi));
  pars_list.register_command("yi", &(traj.yi));

  pars_list.register_command("cx", &(traj.cx));
  pars_list.register_command("cy", &(traj.cy));

  pars_list.register_command("fv", &(robot.follow_v));
  pars_list.register_command("fk", &(robot.follow_k));

  //pars_list.register_command("fk", &(robot.i_lambda));
  pars_list.register_command("kt", &(traj.ktheta));
  //pars_list.register_command("ssid", ssid, max_wifi_str);
  //pars_list.register_command("pass", password, max_wifi_str);

  udp_commands.init(process_command, serial_write);
  
  serial_commands.init(process_command, serial_write);

  robot.pchannels = &serial_commands;

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);
  Serial.printf("Serial Begin");  

  LittleFS.begin();

  float control_interval = 0.04;  // In seconds
  
  // All wheeel PID controllers share the same parameters
  wheel_PID_pars.Kf = 0.2;
  wheel_PID_pars.Kc = 0.167;
  wheel_PID_pars.Ki = 1;
  wheel_PID_pars.Kd = 0;
  wheel_PID_pars.Kfd = 0;
  wheel_PID_pars.dt = control_interval;
  wheel_PID_pars.dead_zone = 0.1;
  int i;
  for (i = 0; i < NUM_WHEELS; i++) {
    robot.PID[i].init_pars(&wheel_PID_pars);
  }

  strcpy(ssid, "TP-Link_8F20");
  strcpy(password, "38911057");

  //strcpy(ssid, "TP-Link_4B12");
  //strcpy(password, "23893481");

  //strcpy(ssid, "TP-Link_28CD");
  //strcpy(password, "49871005");

  load_commands(pars_fname, serial_commands);

// Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);
 
  // Start WiFi with supplied parameters
  WiFi.begin(ssid, password);

  //init_OTA();
  
  robot.max_w1e = 0;
  robot.max_w2e = 0;
  robot.max_w3e = 0;
  robot.max_w4e = 0;

  set_interval(control_interval);    // In seconds
  init_control(robot);
  Serial.printf("End Setup()");  
}

void loop() 
{
  if (WiFi.connected() && !ip_on) {
  //if (WiFi.connected()) {
    // Connection established
    serial_commands.send_command("msg", (String("Pico W is connected to WiFi network with SSID ") + WiFi.SSID()).c_str());
 
    // Print IP Address
    ip_on = Udp.begin(localUdpPort);
    Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  }

  if (ip_on) {
    //ArduinoOTA.handle();

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int i;
      udp_on = 1;
      // receive incoming UDP packets

      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0) {
        UdpInPacket[len] = 0;
      }
      //Serial.printf("UDP packet contents (as string): %s\n", UdpInPacket);

      for (i = 0; i < len; i++) {
        udp_commands.process_char(UdpInPacket[i]);
        //Serial.write(UdpInPacket[i]);
      }
    }      
  }

  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read
  
    b = Serial.read();    
    serial_commands.process_char(b);
    //Serial.write(b);
  }  


  if (load_pars_requested) {
     load_commands(pars_fname, serial_commands);
     load_pars_requested = false;
  }

  // Do this only every "interval" microseconds 
  uint32_t now = micros();
  uint32_t delta = now - last_cycle; 
  if (delta >= interval) {
  // In this case the trigger is a command (serial or UDP)
  //if (robot.control_event) {
    robot.control_event = false;
    loop_micros = micros();
    last_cycle = now;
    //last_cycle += interval;

    // Read and process sensors
     
    robot.odometry();
    robot.localization();
    robot.battery_voltage = 7.4; // if it could not be measured...
    
    //robot.IRLine_Front.calcCrosses();

    //robot.debug1 = robot.IRLine_Back.pos_left;
    //robot.debug2 = robot.IRLine_Back.pos_right;
    //robot.debug3 = robot.IRLine_Back.dist_center;
    //robot.debug3 = 0;
    //robot.debug4 = robot.IRLine_Back.pos_right;
    

    // Control the robot here by choosing:
    //   PWM_1_req and PWM_1_req  when robot.control_mode = cm_pwm
    //   v1_req and v2_req        when robot.control_mode = cm_pid
    //   v_req and w_req          when robot.control_mode = cm_kinematics
    control(robot);

    // Calc outputs
    //robot.accelerationLimit();
    robot.v = robot.v_req;
    robot.vn = robot.vn_req;
    robot.w = robot.w_req;

    serial_commands.send_command("v", robot.v);
    serial_commands.send_command("vn", robot.vn);
    serial_commands.send_command("w", robot.w);
    
    robot.calcMotorsVoltage();

    digitalWrite(LED_BUILTIN, robot.led);
    
    // Debug information
    serial_commands.send_command("dte", delta);

    serial_commands.send_command("u1", robot.u1);
    serial_commands.send_command("u2", robot.u2);
    serial_commands.send_command("u3", robot.u3);
    serial_commands.send_command("u4", robot.u4);    

    //serial_commands.send_command("e1", robot.enc1);
    //serial_commands.send_command("e2", robot.enc2);

    //serial_commands.send_command("Vbat", robot.battery_voltage);

    //serial_commands.send_command("ve", robot.ve);
    //serial_commands.send_command("we", robot.we);

    //serial_commands.send_command("w1", robot.w1e);
    //serial_commands.send_command("w2", robot.w2e);

    serial_commands.send_command("sl", robot.solenoid_PWM);

    serial_commands.send_command("mode", robot.control_mode);

    serial_commands.send_command("st", robot.pfsm->state);

    serial_commands.send_command("IP", WiFi.localIP().toString().c_str());
    
    //serial_commands.send_command("pr", robot.IRLine.pos_right);
    //serial_commands.send_command("pl", robot.IRLine.pos_left);

    serial_commands.send_command("xe", robot.xe);
    serial_commands.send_command("ye", robot.ye);

    serial_commands.send_command("te", robot.thetae);

    serial_commands.send_command("dc", cycle_count -  last_cycle_count);

    if (debug_mode == true){
      serial_commands.send_command("d1", robot.debug1);
      serial_commands.send_command("d2", robot.debug2);
      serial_commands.send_command("d3", robot.debug3);
      serial_commands.send_command("d4", robot.debug4);
    }  

    pars_list.send_sparse_commands(serial_commands);

    Serial.print(" cmd: ");
    Serial.print(serial_commands.frame.command);
    Serial.print("; ");
      
    debug = serial_commands.out_count;
    serial_commands.send_command("dbg", debug); 
    serial_commands.send_command("loop", micros() - loop_micros);  
     
    serial_commands.flush();   
    Serial.println();

    http_ota.handle();
  }
    
}

