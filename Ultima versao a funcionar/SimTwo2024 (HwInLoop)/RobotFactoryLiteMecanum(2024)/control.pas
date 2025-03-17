//
//

const
 NumMachines = 4;
 NumParts = 4;

type
  TControlMode = (cmManual, cmSerial, cmUDP, cmZMQ);

  TMachine = record
    Xi, Yi, Xo: double;
  end;

  TRobot = record
    IKMat: Matrix;

    enc1, enc2, enc3, enc4: integer;
    max_enc1, max_enc2, max_enc3, max_enc4: integer;
    sens_line: array[0..9] of integer;
    touch_sensor: integer;

    iMicroSwitch, iSolenoid1, iMotor1, iMotor2, iMotor3, iMotor4: integer;

    a, b, r: double;
    X, Y, Theta: double;
    Xe, Ye, Te: double;
    vx, vy: double;

    V, Vn, W: double;
    w1, w2, w3, w4: double;
    u1, u2, u3, u4: double;

    SolenoidPWM: integer;
    State: integer;
  end;

  TChannel = record
   command, text, index_str, array_command: string;
   value: single;
   index: integer;
  end;

  TGChannelState = (tgcs_wait_for_command, tgcs_reading_data);

  TGChannels = record
    serialData, buffer: string;
    count: integer;
    frame_command, frame_text: pchar;

    Channel: TChannel;
    state: TGChannelState;
  end;


// Global Variables
var
  irobot, NumRobots: integer;
  t: double;

  PIDactive,DebugON: Boolean;
  rfid_tag: integer;

  ControlMode: TControlMode;

  //iSolenoid, iMicroSwitch, iRFID: integer;


  //FrameDigits: integer;
  //channel: char;
  //frame, frameSource: integer;
  //frameData: string;

  part_pos_z: double;
  Machines: array[0..NumMachines - 1] of TMachine;
  machine_pos_x, machine_pos_y: double;

  Robot: TRobot;
  Channels: TGChannels;
  count: integer;
  timer_time, max_timer_time, min_timer_time, last_timer_time, cycle_count: integer;

procedure process_command(channel: TChannel); forward;


function isalpha(c: char): boolean;
begin
  result := false;
  if ((c >= 'A') and (c <= 'Z')) or
     ((c >= 'a') and (c <= 'z')) then result := true;
end;

procedure process_char(var GChannels: TGChannels; b: char);
var len, i: integer;
begin
  with GChannels do begin
    if (state = tgcs_wait_for_command) and isalpha(b) then begin // A command allways starts with a letter
      state := tgcs_reading_data;
      buffer := b;
      count := 1;

    end else if (state = tgcs_reading_data) and (b = chr($08))  then begin // BS (Backspace key received)
      if (count > 0) then begin
        dec(count);
        Delete(buffer, count, 1);
      end;

    end else if (state = tgcs_reading_data) and ((b = chr($0A)) or (b = chr($0D)) or (b = ';'))  then begin // LF or CR (enter key received) or ';'
      // Now we can process the buffer
      if (count > 0) then begin
        channel.command := '';
        channel.text := '';
        channel.index_str := '0'

        // Find the first space to separate the command from the text/value
        len := length(buffer);
        i := 1;
        while i <= len do begin
          if (buffer[i] = ':') or (buffer[i] = ' ') then begin  // If the first space found or a ':'
            channel.command := copy(buffer, 1, i - 1);  // We have the Command
            inc(i);
            while (i <= len) do begin // we should skip extra spaces
              if (buffer[i] <> ' ') then break;
              inc(i);
            end;
            channel.text := copy(buffer, i, len);  // Now we have also the "text"
          end;
          inc(i);
        end;
        // for the numerical parameter try to get the value from the text
        channel.value := StrToFloatDef(channel.text, 0);

        process_command(channel); // Do something with the pair (command, value)


        // Reset the buffer
        buffer := '';
      end;
      state := tgcs_wait_for_command;


    end else if (state = tgcs_reading_data) then begin // A new char can be read
      buffer := buffer + b;  // Store byte in the buffer
      inc(count);

    end;
  end;
end;

function build_command(channel: string; value: double): string;
begin
  result := format('%s %.6g;', [channel, value]);
end;

//Procedure, functions and Routines


procedure process_command(channel: TChannel);
begin
  //exit;
  if channel.command = 'loop' then begin
    if DebugON=TRUE then SetRCValue(27, 4, format('%.3f',[channel.value / 1000]));

  end else if channel.command = 'u1' then begin
    Robot.u1 :=  channel.value;
    if DebugON=TRUE then SetRCValue(2, 11, format('%.3f',[Robot.u1]));
  end else if channel.command = 'u2' then begin
    Robot.u2 :=  channel.value;
    if DebugON=TRUE then SetRCValue(3, 11, format('%.3f',[Robot.u2]));

  end else if channel.command = 'u3' then begin
    Robot.u3 :=  channel.value;
    if DebugON=TRUE then SetRCValue(4, 11, format('%.3f',[Robot.u3]));

  end else if channel.command = 'u4' then begin
    Robot.u4 :=  channel.value;
    if DebugON=TRUE then SetRCValue(5, 11, format('%.3f',[Robot.u4]));

  end else if channel.command = 'v' then begin
    if DebugON=TRUE then SetRCValue(7, 11, format('%.3f',[channel.value]));

  end else if channel.command = 'vn' then begin
    if DebugON=TRUE then SetRCValue(8, 11, format('%.3f',[channel.value]));

  end else if channel.command = 'w' then begin
    if DebugON=TRUE then SetRCValue(9, 11, format('%.3f',[channel.value]));

  end else if channel.command = 'd1' then begin
    if DebugON=TRUE then SetRCValue(11, 11, format('%.3f',[channel.value]));

  end else if channel.command = 'd2' then begin
    if DebugON=TRUE then SetRCValue(12, 11, format('%.3f',[channel.value]));

  end else if channel.command = 'd3' then begin
    if DebugON=TRUE then SetRCValue(13, 11, format('%.3f',[channel.value]));

  end else if channel.command = 'd4' then begin
    if DebugON=TRUE then SetRCValue(14, 11, format('%.3f',[channel.value]));

  end else if channel.command = 'sl' then begin
    Robot.SolenoidPWM := round(channel.value);

  end else if channel.command = 'st' then begin
    Robot.state := round(channel.value);

  end else if channel.command = 'xe' then begin
    Robot.Xe := channel.value;

  end else if channel.command = 'ye' then begin
    Robot.Ye := channel.value;

  end else if channel.command = 'te' then begin
    Robot.Te := channel.value;

  end else if channel.command = 'IP' then begin
    SetRCValue(20, 2, format('%s',[channel.text]));

  end else if channel.command = 'dc' then begin
    SetRCValue(29, 6, format('%.3f',[channel.value]));
  end;

end;


function max(a, b: double): double;
begin
  if a > b then result := a else result := b;
end;

function min(a, b: double): double;
begin
  if a < b then result := a else result := b;
end;


//-------------------------------------------------------------------
// Warehouses, Machines and parts functions
procedure SetPartPos(PartIdx: integer; newX, newY, newTheta: double);
begin
  SetRobotPos(1 + PartIdx, newX, newY, part_pos_z, newTheta);
end;


function GetPartType(PartIdx: integer): string;
begin
  result := GetShellTag(1 + PartIdx, 5);
end;

procedure SetPartType(PartIdx: integer; PartType: integer);
begin
  if PartType = 1 then begin
    SetShellTag(1 + PartIdx, 5, '1');
    SetSolidColor(1 + PartIdx, 0, 200, 0, 0);
  end else if PartType = 2 then begin
    SetShellTag(1 + PartIdx, 5, '2');
    SetSolidColor(1 + PartIdx, 0, 0, 200, 0);
  end else if PartType = 3 then begin
    SetShellTag(1 + PartIdx, 5, '3');
    SetSolidColor(1 + PartIdx, 0, 0, 100, 200);
  end;
end;

function RobotMachineInDist(M: TMachine): double;
var P: TPoint3D;
begin
  P := GetSolidPos(0, 0);
  result := dist(M.Xi - P.x, M.Yi - P.y);
end;

function PartMachineOutDist(M: TMachine; Pi: integer): double;
var P: TPoint3D; // Part Position
    i: integer;
    d: double;
begin
  result := 1000;
  for i := 0 to numParts - 1 do begin
    //if i = Pi then continue;
    P := GetSolidPos(1 + i, 0);
    d := dist(M.Xo - P.x, M.Yi - P.y);
    if d < result then result := d;
  end;
end;

function PartMachineInDist(M: TMachine; Pi: integer): double;
var P: TPoint3D;
begin
  P := GetSolidPos(1 + Pi, 0);
  result := dist(M.Xi - P.x, M.Yi - P.y);
end;


procedure MachineParts;
var pi, mi, ci: integer;
    c: string;
begin
  for mi := 0 to NumMachines - 1 do begin
    for Pi := 0 to NumParts - 1 do begin
      if (PartMachineInDist(Machines[mi], Pi) < 0.03) and
         (RobotMachineInDist(Machines[mi]) > 0.17) and
         (PartMachineOutDist(Machines[mi], Pi) > 0.07) then begin
        SetPartPos(Pi, Machines[mi].Xo, Machines[mi].Yi, rad(90));
        c := GetPartType(Pi);
        if c = '1' then ci := 2;
        if c = '2' then ci := 3;
        SetPartType(Pi, ci);
      end;
    end;
  end;
end;

//------------------------------------------------------------------------------
// Robot Control Section

procedure ManualControl;
begin
  // Initialize to default (not pressed) values
  Robot.V := 0;
  Robot.Vn := 0;
  Robot.w := 0;

  // Check each key press independently
  if KeyPressed(vk_down)  then Robot.V := -0.05;
  if KeyPressed(vk_left)  then Robot.Vn := 0.05;
  if KeyPressed(vk_right) then Robot.Vn := -0.05;
  if KeyPressed(vk_up)    then Robot.V := 0.05;
  if KeyPressed(ord('A')) then Robot.w := 0.5;
  if KeyPressed(ord('D')) then Robot.w := -0.5;
  if KeyPressed(vk_Prior) then Robot.SolenoidPWM := 100;
  if KeyPressed(vk_Next)  then Robot.SolenoidPWM := 0;

end;

function PreparePoseData: string;
var s: string;

begin
  s := s + build_command('xr', GetRCValue(12, 3));
  s := s + build_command('yr', GetRCValue(13, 3));
  s := s + build_command('tr', rad(GetRCValue(14, 3)));
  result := s;
end;

function PrepareData: string;
var s: string;
    i: integer;
begin
  s := '';
  s := build_command('ac', cycle_count);
  // Eventuaol new state request
  if RCButtonPressed(20, 5) then begin
    s := build_command('st', GetRCValue(20, 6));
  end;
  if RCButtonPressed(21, 5) then begin
    s := build_command('st', GetRCValue(21, 6));
  end;

  s := s + build_command('e1', robot.enc1);
  s := s + build_command('e2', robot.enc2);
  s := s + build_command('e3', robot.enc3);
  s := s + build_command('e4', robot.enc4);

  for i:= 0 to 9 do begin
    s := s + build_command('L' + IntToStr(i), round(Robot.sens_line[i] / 10));
  end;
  s := s + build_command('di', robot.touch_sensor);
  s := s + build_command('go', 1);
  result := s;
end;

procedure mecIK();
var input, output : Matrix;
begin
  MInit(input, 3, 1);
  MInit(output, 4, 1);

  Msetv(input, 0, 0, Robot.V);
  Msetv(input, 1, 0, Robot.Vn);
  Msetv(input, 2, 0, Robot.W);

  output := MmultReal(Mmult(Robot.IKMat, input), (1 / Robot.r));

  Robot.w1 := Mgetv(output, 0, 0);
  Robot.w2 := Mgetv(output, 1, 0);
  Robot.w3 := Mgetv(output, 2, 0);
  Robot.w4 := Mgetv(output, 3, 0);
end;


procedure ActuateRobot();
begin

  if ControlMode = cmManual then begin
    mecIK();
    SetAxisSpeedRef(iRobot, Robot.iMotor1, Robot.w1);
    SetAxisSpeedRef(iRobot, Robot.iMotor2, Robot.w2);
    SetAxisSpeedRef(iRobot, Robot.iMotor3, Robot.w3);
    SetAxisSpeedRef(iRobot, Robot.iMotor4, Robot.w4);
  end else begin
    SetAxisVoltageRef(iRobot, Robot.iMotor1, Robot.u1);
    SetAxisVoltageRef(iRobot, Robot.iMotor2, Robot.u2);
    SetAxisVoltageRef(iRobot, Robot.iMotor3, Robot.u3);
    SetAxisVoltageRef(iRobot, Robot.iMotor4, Robot.u4);
  end;

  SetSensorVin(irobot, Robot.iSolenoid1, Robot.SolenoidPWM / 100);

  SetRCValue(20, 4, format('%d',[Robot.State]));
  SetRCValue(22, 4, format('%d',[round(Robot.SolenoidPWM)]));
  SetRCValue(23, 4, format('%5.2f',[robot.V]));
  SetRCValue(24, 4, format('%5.2f',[robot.Vn]));
  SetRCValue(25, 4, format('%5.2f',[robot.W]));


  robot.Vx := GetRobotVx(irobot);
  robot.Vy := GetRobotVy(irobot);

  (*
  robot.X := GetRobotX(irobot);
  robot.Y := GetRobotY(irobot);
  robot.Theta := GetRobotTheta(irobot);

  SetRCValue(12, 2, format('%5.2f',[robot.X]));
  SetRCValue(13, 2, format('%5.2f',[robot.Y]));
  SetRCValue(14, 2, format('%5.2f',[deg(robot.Theta)]));
  *)

  SetRCValue(12, 2, format('%5.3f',[robot.Xe]));
  SetRCValue(13, 2, format('%5.3f',[robot.Ye]));
  SetRCValue(14, 2, format('%5.3f',[deg(robot.Te)]));

  inc(count);
end;


procedure TimerEvent;
var StrPacket: TStringList;
    txt, s, ip: string;
    i, c: integer;
    v: double;

begin
  t := t + ScriptPeriod();

  //if RCButtonPressed(15, 1) then begin
  MachineParts();
  //end;

  // Set Robot position
  if RCButtonPressed(11, 3) then begin
    SetRobotPos(0, GetRCValue(12, 3), GetRCValue(13, 3), 0.01, rad(GetRCValue(14, 3)));
    s := PreparePoseData;
    //ip := GetRCText(1, 2);
    //WriteUDPData(ip, 4224, s);
    WriteComPort(s);
    robot.max_enc1 := 0;
    robot.max_enc2 := 0;
    robot.max_enc3 := 0;
    robot.max_enc4 := 0;
  end;

  // Parts UI
  for i:= 0 to 3 do begin
    if RCButtonPressed(11, 4 + i) then begin
      SetRobotPos(1 + i, GetRCValue(12, 4 + i), GetRCValue(13, 4 + i), 0, rad(GetRCValue(14, 4 + i)));
    end;
    if RCButtonPressed(15, 4 + i) then begin
      c := StrToIntDef(GetPartType(i), 0) + 1;
      if c > 3 then c := 1;
      SetPartType(i, c);
    end;
  end;

  // Read line sensors
  for i := 0 to 4 do begin
    robot.sens_line[i] := round(1023 * GetSensorVal(0, i));
    SetRCValue(5 + i, 2, format('%d',[robot.sens_line[i]]));
  end;
  for i := 5 to 9 do begin
    robot.sens_line[i] := round(1023 * GetSensorVal(0, i));
    SetRCValue(i, 3, format('%d',[robot.sens_line[i]]));
  end;

  robot.touch_sensor := 0;
  if GetSensorVal(0, robot.iMicroSwitch) > 0 then robot.touch_sensor := 1;
  SetRCValue(27, 2, format('%d', [robot.touch_sensor]));

  //rfid_tag := round(GetSensorVal(0, iRFID));
  //SetRCValue(10, 2, format('%d', [rfid_tag]));

  robot.enc1 := GetAxisOdo(0, 0);
  robot.enc2 := GetAxisOdo(0, 1);
  robot.enc3 := GetAxisOdo(0, 2);
  robot.enc4 := GetAxisOdo(0, 3);

  If abs(robot.enc1) > abs(robot.max_enc1) then robot.max_enc1 := robot.enc1;
  If abs(robot.enc2) > abs(robot.max_enc2) then robot.max_enc2 := robot.enc2;
  If abs(robot.enc3) > abs(robot.max_enc3) then robot.max_enc3 := robot.enc3;
  If abs(robot.enc4) > abs(robot.max_enc4) then robot.max_enc4 := robot.enc4;

  if DebugON=TRUE then begin
    (*
    SetRCValue(22, 2, format('%d', [robot.enc1]));
    SetRCValue(23, 2, format('%d', [robot.enc2]));
    SetRCValue(24, 2, format('%d', [robot.enc3]));
    SetRCValue(25, 2, format('%d', [robot.enc4]));
    *)
    SetRCValue(22, 2, format('%d', [robot.max_enc1]));
    SetRCValue(23, 2, format('%d', [robot.max_enc2]));
    SetRCValue(24, 2, format('%d', [robot.max_enc3]));
    SetRCValue(25, 2, format('%d', [robot.max_enc4]));

  end;


  if RCButtonPressed(2, 3) then begin
    ControlMode := cmManual;
    SetRCValue(2, 2, 'Manual');
    PIDactive := TRUE;
    SetMotorControllerState(iRobot, Robot.iMotor1, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor2, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor3, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor4, PIDactive);
    SetMotorControllerMode(iRobot, Robot.iMotor1, 'pidspeed');
    SetMotorControllerMode(iRobot, Robot.iMotor2, 'pidspeed');
    SetMotorControllerMode(iRobot, Robot.iMotor3, 'pidspeed');
    SetMotorControllerMode(iRobot, Robot.iMotor4, 'pidspeed');
  end else if RCButtonPressed(2, 4) then begin
    ControlMode := cmSerial;
    SetRCValue(2, 2, 'Serial');
    PIDactive := FALSE;
    SetMotorControllerState(iRobot, Robot.iMotor1, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor2, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor3, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor4, PIDactive);
  end else if RCButtonPressed(2, 5) then begin
    ControlMode := cmUDP;
    SetRCValue(2, 2, 'UDP');
    PIDactive := FALSE;
    SetMotorControllerState(iRobot, Robot.iMotor1, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor2, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor3, PIDactive);
    SetMotorControllerState(iRobot, Robot.iMotor4, PIDactive);
  end else if RCButtonPressed(2, 6) then begin
    ControlMode := cmZMQ;
    SetRCValue(2, 2, 'ZMQ');
  end;

  case ControlMode of
    cmManual: begin
       ManualControl;
     end;
    cmSerial: begin
       s := PrepareData();
       WriteComPort(s);
       //writeln(s);
     end;
    cmUDP: begin
       s := PrepareData();
       ip := GetRCText(1, 2);
       WriteUDPData(ip, 4224, s);
       //Writeln(s);
     end;
    cmZMQ: begin
       s := PrepareData();
       ip := GetRCText(1, 2);
       PubZMQData(s, false);
       ///writeln(s);
     end;
  end;

end;



procedure Control;
var s: string;
    i, delta_time: integer;
begin
  cycle_count := cycle_count +1;
  if event = 'timer' then begin
    timer_time := GetTickCount();
    if DebugOn = TRUE then begin
      delta_time := timer_time - last_timer_time;
      SetRCValue(29, 2, format('%d',[delta_time]));
      last_timer_time := timer_time;
      if cycle_count > 50 then begin
        if delta_time < min_timer_time then min_timer_time := delta_time;
        if delta_time > max_timer_time then max_timer_time := delta_time;
        SetRCValue(30, 2, format('%d',[max_timer_time]));
        SetRCValue(31, 2, format('%d',[min_timer_time]));
      end;
    end;
    TimerEvent();

  end else if event = 'serial_data' then begin
    //SetRCValue(1, 3, format('%d',[(GetTickCount() - timer_time)]));
    s := ReadComPort();
    if s <> '' then WriteLn(S);
    for i := 1 to Length(s) do begin
      process_char(channels, s[i]);
    end;

  end else if event = 'udp_data' then begin
    //SetRCValue(1, 3, format('%d',[(GetTickCount() - timer_time)]));
    s := ReadUDPData();
    for i := 1 to Length(s) do begin
      process_char(channels, s[i]);
    end;
   // if s <> '' then WriteLn(S);

  end else if event = 'zmq_data' then begin
    //SetRCValue(1, 3, format('%d',[(GetTickCount() - timer_time)]));
    s := ReadZMQData();
    for i := 1 to Length(s) do begin
      process_char(channels, s[i]);
    end;
    //if s <> '' then WriteLn(S);

  end else begin
    writeln(event)
  end;

  if RCButtonPressed(29, 3) then begin
    max_timer_time := 0;
    min_timer_time := 10000;
    cycle_count := 0;
  end;

  ActuateRobot();

end;


procedure Initialize;
var
  temp: double;
begin

  ClearButtons();
  iRobot := GetRobotIndex('mecanum');


  Robot.isolenoid1  := GetSensorIndex(iRobot, 'solenoid1');
  Robot.iMicroSwitch  := GetSensorIndex(iRobot, 'MicroSwitch');
  Robot.iMotor1 := GetAxisIndex(iRobot, 'LeftAxisFront', 0);
  Robot.iMotor2 := GetAxisIndex(iRobot, 'RightAxisFront', 0);
  Robot.iMotor3 := GetAxisIndex(iRobot, 'LeftAxisBack', 0);
  Robot.iMotor4 := GetAxisIndex(iRobot, 'RightAxisBack', 0);

  Robot.State := -1;

  Robot.a := 0.25 / 2 - 0.05 ;
  Robot.b := 0.155 / 2 + 0.015 ;

  Robot.r := 0.065 / 2;

  PIDactive := TRUE;
  SetMotorControllerState(iRobot, Robot.iMotor1, PIDactive);
  SetMotorControllerState(iRobot, Robot.iMotor2, PIDactive);
  SetMotorControllerState(iRobot, Robot.iMotor3, PIDactive);
  SetMotorControllerState(iRobot, Robot.iMotor4, PIDactive);
  SetMotorControllerMode(iRobot, Robot.iMotor1, 'pidspeed');
  SetMotorControllerMode(iRobot, Robot.iMotor2, 'pidspeed');
  SetMotorControllerMode(iRobot, Robot.iMotor3, 'pidspeed');
  SetMotorControllerMode(iRobot, Robot.iMotor4, 'pidspeed');

  MInit(Robot.IKMat, 4, 3);
  temp := Robot.a + Robot.b;

  Msetv(Robot.IKMat, 0, 0, 1);  Msetv(Robot.IKMat, 0, 1, -1);  Msetv(Robot.IKMat, 0, 2, -temp);
  Msetv(Robot.IKMat, 1, 0, 1);  Msetv(Robot.IKMat, 1, 1,  1);  Msetv(Robot.IKMat, 1, 2,  temp);
  Msetv(Robot.IKMat, 2, 0, 1);  Msetv(Robot.IKMat, 2, 1,  1);  Msetv(Robot.IKMat, 2, 2, -temp);
  Msetv(Robot.IKMat, 3, 0, 1);  Msetv(Robot.IKMat, 3, 1, -1);  Msetv(Robot.IKMat, 3, 2,  temp);

  t := 0;
  ControlMode := cmManual;
  SetRCValue(2, 2, 'Manual');

  //ControlMode := cmSerial;
  //SetRCValue(2, 2, 'Serial');

  //ControlMode := cmZMQ;
  //SetRCValue(2, 2, 'ZMQ');

  //SetRCValue(11, 2, inttostr(ord(isHexDigit('A'))));
  SetPartType(0, 1);
  SetPartType(1, 1);
  SetPartType(2, 2);
  SetPartType(3, 3);

  machine_pos_x := 0.720 / 2;
  machine_pos_y := 0.075;
  part_pos_z := 0.0021;

  with Machines[0] do begin
    Xi := -machine_pos_x - 0.045;
    Yi := 0;
    //SetPartPos(0, Xi, Yi, rad(-90));
    Xo := -machine_pos_x + 0.045;
    //SetPartPos(1, Xo, Yi, rad(90));
  end;

  with Machines[1] do begin
    Xi := -machine_pos_x - 0.045;
    Yi := -2*machine_pos_y;
    //SetPartPos(1, Xi, Yi, rad(-90));
    Xo := -machine_pos_x + 0.045;
    //SetPartPos(1, Xo, Yi, rad(90));
  end;

  with Machines[2] do begin
    Xi := machine_pos_x - 0.045;
    Yi := 2*machine_pos_y;
    //SetPartPos(2, Xi, Yi, rad(-90));
    Xo := machine_pos_x + 0.045;
    //SetPartPos(2, Xo, Yi, rad(90));
  end;

  with Machines[3] do begin
    Xi := machine_pos_x - 0.045;
    Yi := 0;
    //SetPartPos(3, Xi, Yi, rad(-90));
    Xo := machine_pos_x + 0.045;
    //SetPartPos(3, Xo, Yi, rad(90));
  end;

  last_timer_time := GetTickCount();
  max_timer_time := 0;
  min_timer_time := 10000;
  cycle_count := 0;

  DebugOn := TRUE;

  robot.max_enc1 := 0;
  robot.max_enc2 := 0;
  robot.max_enc3 := 0;
  robot.max_enc4 := 0;
end;



