const
 NumMachines = 4;
 NumParts = 4;

type
  TControllerMode = (ceVoltage, cePIDRobotSpeeds);
  TControlMode = (cmManual, cmRemote, cmScript);

  TMecanumRobot = record
    iRobot, iLaser: Integer;
    iMotor1, iMotor2, iMotor3, iMotor4 : integer;
    isolenoid1, iMicroSwitch: integer;

    pose : TState2D;
    a, b, r : double;

    Vx, Vy, V, Vn, w : double;
    V_ref, Vn_ref, w_ref : double;

    w1, w2, w3, w4 : double;
    w1_ref, w2_ref, w3_ref, w4_ref : double;

    e1, e2, e3, e4 : longInt;
    TouchSensor: integer;
    SolenoidActive: integer;

    IKMat: Matrix;

    ControllerMode: TControllerMode;
    state: string;
  end;

  TMachine = record
    Xi, Yi, Xo: double;
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



// Global Variables Here
var
  Channels: TGChannels;
  //Robot
  RC : TMecanumRobot;

  part_pos_z: double;
  Machines: array[0..NumMachines - 1] of TMachine;
  machine_pos_x, machine_pos_y: double;

  //Auxiliary variables
  PIDactive: Boolean;
  //state: integer;
  Logging: boolean;
  //currentTarget : TState2D;

  linearSpeed, angularSpeed, poseThreshold, angleThreshold, maxdV : double;
  dt : double;
  log : TStringList;

  ControlMode : TControlMode;
  timer_time: integer;

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
            while (buffer[i] = ' ') and (i <= len) do begin // we should skip extra spaces
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


//----------------------------------------------------------------------
// Part management code

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

function PartMachineInDist(M: TMachine; Pi: integer): double;
var P: TPoint3D;
begin
  P := GetSolidPos(1 + Pi, 0);
  result := dist(M.Xi - P.x, M.Yi - P.y);
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

// End of Part management code
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// GChannels events


procedure process_command(channel: TChannel);
begin
  //exit;
  if channel.command = 'loop' then begin
    SetRCValue(1, 2, format('%.3f',[channel.value / 1000]));


  end else if channel.command = 'w' then begin
    RC.w_ref :=  channel.value;
    //SetRCValue(2, 11, format('%.3f',[RC.w_ref]));

  end else if channel.command = 'v' then begin
    RC.V_ref :=  channel.value;
    //SetRCValue(3, 11, format('%.3f',[RC.V_ref]));

  end else if channel.command = 'vn' then begin
    RC.Vn_ref :=  channel.value;
    //SetRCValue(2, 11, format('%.3f',[RC.Vn_ref]));

  end else if channel.command = 'st' then begin
    RC.state :=  channel.text;
    SetRCValue(3, 2, format('%d',[RC.state]));
  end;

end;


function PrepareLaserSweepPackage(LaserScanRays: Matrix) : string;
var i, nrows : integer;
    temp : string;
begin
  //d := (mgetv(LaserScanRays, 0, 0));
  temp := '';
  nrows := MNumRows(LaserScanRays);

  for i := 0 to nrows - 1 do begin
    temp := temp + Format('%.4g', [Mgetv(LaserScanRays, i, 0)]) + ',';
  end;

  Delete(temp, Length(temp), 1);
  //writeln(temp);
  Result:=temp;
end;


procedure updateRobotsState();
var ct, st: double;
begin
  RC.pose := GetRobotPos2D(RC.iRobot);
  RC.Vx := GetRobotVx(RC.iRobot);
  RC.Vy := GetRobotVy(RC.iRobot);
  RC.W := GetRobotW(RC.iRobot);

  RC.W1 := GetAxisSpeed(RC.iRobot, 0);
  RC.W2 := GetAxisSpeed(RC.iRobot, 1);
  RC.W3 := GetAxisSpeed(RC.iRobot, 2);
  RC.W4 := GetAxisSpeed(RC.iRobot, 3);

  st := sin(RC.pose.angle);
  ct := cos(RC.pose.angle);

  RC.V := RC.Vx * ct + RC.Vy * st;
  RC.Vn := -RC.Vx * st + RC.Vy * ct;

  SetRCValue(1, 2, Format('%.3f', [RC.V]));
  SetRCValue(2, 2, Format('%.3f', [RC.Vn]));
  SetRCValue(3, 2, Format('%.3f', [RC.w]));

  SetRCValue(12, 2, Format('%.2g', [RC.pose.x]));
  SetRCValue(13, 2, Format('%.2g', [RC.pose.y]));
  SetRCValue(14, 2, Format('%.4g', [deg(RC.pose.angle)]));

  SetRCValue(5, 2, Format('%.3f', [RC.w1]));
  SetRCValue(6, 2, Format('%.3f', [RC.w2]));
  SetRCValue(7, 2, Format('%.3f', [RC.w3]));
  SetRCValue(8, 2, Format('%.3f', [RC.w4]));
end;


procedure mecIK();
var input, output : Matrix;
begin
  MInit(input, 3, 1);
  MInit(output, 4, 1);

  Msetv(input, 0, 0, RC.V_ref);
  Msetv(input, 1, 0, RC.Vn_ref);
  Msetv(input, 2, 0, RC.W_ref);

  output := MmultReal(Mmult(RC.IKMat, input), (1 / RC.r));

  RC.w1 := Mgetv(output, 0, 0);
  RC.w2 := Mgetv(output, 1, 0);
  RC.w3 := Mgetv(output, 2, 0);
  RC.w4 := Mgetv(output, 3, 0);
end;


procedure resetRobot();
begin
  SetRobotPos(RC.iRobot, -0.6, -0.39, 0.01, 1.6);
  dt := 0;
end;


procedure actuateRobot();
begin
  mecIK();

  SetAxisSpeedRef(RC.iRobot, RC.iMotor1, RC.w1);
  SetAxisSpeedRef(RC.iRobot, RC.iMotor2, RC.w2);
  SetAxisSpeedRef(RC.iRobot, RC.iMotor3, RC.w3);
  SetAxisSpeedRef(RC.iRobot, RC.iMotor4, RC.w4);

  SetSensorVin(RC.irobot, RC.iSolenoid1, RC.SolenoidActive);
end;


procedure logDataToCSV();
begin
  log.Add(FloatToStr(RC.pose.x) + ',' + FloatToStr(RC.pose.y) + ',' +
  FloatToStr(RC.pose.angle) + ',' + FloatToStr(dt));
end;


procedure saveToFile(const FileName : String);
begin
  log.SaveToFile(FileName);
  log.clear;
end;



procedure prepareLaserMsg(var StrPacket : TStringList);
var
  LaserValues : Matrix;
begin
  LaserValues := GetSensorValues(RC.iRobot, RC.iLaser);
  StrPacket.add('lidar');
  StrPacket.add(PrepareLaserSweepPackage(LaserValues));
  StrPacket.add('');
  MatrixToRange(26, 1, LaserValues);
end;


procedure ManualControl;
begin
  // Initialize to default (not pressed) values
  RC.V_ref := 0;
  RC.Vn_ref := 0;
  RC.w_ref := 0;

  // Check each key press independently
  if KeyPressed(vk_down)  then RC.V_ref := -0.3;
  if KeyPressed(vk_left)  then RC.Vn_ref := 0.3;
  if KeyPressed(vk_right) then RC.Vn_ref := -0.3;
  if KeyPressed(vk_up)    then RC.V_ref := 0.3;
  if KeyPressed(ord('A')) then RC.w_ref := 1.5;
  if KeyPressed(ord('D')) then RC.w_ref := -1.5;
  if KeyPressed(vk_Prior) then RC.SolenoidActive := 1;
  if KeyPressed(vk_Next)  then RC.SolenoidActive := 0;

end;



procedure ScriptControl;
begin
  // TODO

end;

procedure SendData;
var mess: string;
begin
  // Encoders
  mess := '';
  mess := mess + build_command('v', RC.V);
  mess := mess + build_command('vn', RC.Vn);
  mess := mess + build_command('w', RC.W);

  //prepareEncodersMsg(OutMess);
  RC.e1 := GetAxisOdo(0, 0);
  RC.e2 := GetAxisOdo(0, 1);
  RC.e3 := GetAxisOdo(0, 2);
  RC.e4 := GetAxisOdo(0, 3);

  mess := mess + build_command('e0', RC.e1);
  mess := mess + build_command('e1', RC.e2);
  mess := mess + build_command('e2', RC.e3);
  mess := mess + build_command('e3', RC.e4);

  // Ground Truth
  mess := mess + build_command('x', RC.pose.x);
  mess := mess + build_command('y', RC.pose.y);
  mess := mess + build_command('ang', RC.pose.angle);
  mess := mess + build_command('sol', RC.SolenoidActive);
  mess := mess + build_command('sw', RC.TouchSensor);

  // LIDAR
  //if (iLaser >= 0) then prepareLaserMsg(StrPacket);
  mess := mess + build_command('loop', GetTickCount() - timer_time);
  WriteUDPData('127.0.0.1', 4224, mess);
end;


procedure DataEvent;
var s: string;
    i: integer;
begin
  s := ReadUDPData();
  for i := 1 to Length(s) do begin
    process_char(channels, s[i]);
  end;
  //if s <> '' then WriteLn(S);end;

  ActuateRobot();
end;


// this procedure is called periodicaly (default: 25 ms)
procedure IimerEvent;
var StrPacket: TStringList;
    txt: string;
    temp : TState2D;
    i, c: integer;
begin
  //if RCButtonPressed(15, 1) then begin
    MachineParts();
  //end;

  UpdateRobotsState();

  //Control mode selection

  if (RCButtonPressed(1,6)) then begin
    SetRCValue(2, 6, 'Manual');
    ControlMode := cmManual;
  end else if (RCButtonPressed(1,7)) then begin
    SetRCValue(2, 6, 'Remote');
    ControlMode := cmRemote;
  end else if (RCButtonPressed(1,8)) then begin
    SetRCValue(2, 6, 'Script');
    ControlMode := cmScript;
  end;

  if RCButtonPressed(11, 3) then begin
    SetRobotPos(0, GetRCValue(12, 3), GetRCValue(13, 3), 0, rad(GetRCValue(14, 3)));
  end;

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

  RC.TouchSensor := 0;
  if GetSensorVal(0, RC.iMicroSwitch) > 0 then RC.TouchSensor := 1;


  // High-level control
  case ControlMode of
    cmManual: begin
      ManualControl();
      ActuateRobot();
    end;
    cmremote: begin
      SendData();
    end;
    cmScript: begin
      ScriptControl();
      ActuateRobot();
    end;
  end;


  //Logging
  if (RCButtonPressed(4, 6)) then begin
    SetRCValue(5, 6, 'On');
    Logging := true;
  end else if (RCButtonPressed(4, 7)) then begin
    SetRCValue(5, 6, 'Off');
    Logging := false;
  end else if (RCButtonPressed(4, 7)) then begin
    saveToFile('trajectory.txt');
  end;

  if Logging then begin
    logDataToCSV();
    dt := dt + 0.025;
    SetRCValue(24, 2, Format('%.2g', [dt]));
  end;

  SetRCValue(7, 6, Format('%d', [GetSensorSampleCount(0, RC.iLaser)]));
  SetRCValue(15, 2, Format('%d', [RC.SolenoidActive]));
  SetRCValue(16, 2, Format('%d', [RC.TouchSensor]));
end;


procedure Control;
begin
  if event = 'timer' then begin
    timer_time := GetTickCount();
    IimerEvent();
  end else if event = 'udp_data' then begin
    //SetRCValue(1, 3, format('%d',[(GetTickCount() - timer_time)]));
    DataEvent();
  end else begin
    writeln(event)
  end;
end;


// this procedure is called once when the script is started
procedure Initialize;
var temp : double;
begin
  RC.iRobot := GetRobotIndex('mecanum');
  RC.iLaser := GetSensorIndex(RC.iRobot, 'ranger2d');
  RC.isolenoid1  := GetSensorIndex(RC.iRobot, 'solenoid1');
  RC.iMicroSwitch  := GetSensorIndex(RC.iRobot, 'MicroSwitch');

  ControlMode := cmManual;
  SetRCValue(2, 6, 'Manual');

  PIDactive := True;
  //state := 0;

  logging := false;

  linearSpeed := 0.3;
  angularSpeed := 2.6;

  maxdV := 5;

  angleThreshold := rad(2.5);
  poseThreshold := 0.0138;

  dt := 0;

  log := TStringList.create;

  log.Add('x,y,theta,time');

   // 4 Wheeled Mecanum

  SetRobotPos(RC.iRobot, -0.6, -0.39, 0.01, 1.6);
  RC.iMotor1 := GetAxisIndex(RC.iRobot, 'LeftAxisFront', 0);
  RC.iMotor2 := GetAxisIndex(RC.iRobot, 'RightAxisFront', 0);
  RC.iMotor3 := GetAxisIndex(RC.iRobot, 'LeftAxisBack', 0);
  RC.iMotor4 := GetAxisIndex(RC.iRobot, 'RightAxisBack', 0);

  RC.a := 0.25 / 2 - 0.05 ;
  RC.b := 0.155 / 2 + 0.015 ;

  RC.r := 0.065;

  RC.ControllerMode := cePIDRobotSpeeds;

  SetMotorControllerState(RC.iRobot, RC.iMotor1, PIDactive);
  SetMotorControllerState(RC.iRobot, RC.iMotor2, PIDactive);
  SetMotorControllerState(RC.iRobot, RC.iMotor3, PIDactive);
  SetMotorControllerState(RC.iRobot, RC.iMotor4, PIDactive);
  SetMotorControllerMode(RC.iRobot, RC.iMotor1, 'pidspeed');
  SetMotorControllerMode(RC.iRobot, RC.iMotor2, 'pidspeed');
  SetMotorControllerMode(RC.iRobot, RC.iMotor3, 'pidspeed');
  SetMotorControllerMode(RC.iRobot, RC.iMotor4, 'pidspeed');

  MInit(RC.IKMat, 4, 3);
  temp := RC.a + RC.b;

  Msetv(RC.IKMat, 0, 0, 1);  Msetv(RC.IKMat, 0, 1, -1);  Msetv(RC.IKMat, 0, 2, -temp);
  Msetv(RC.IKMat, 1, 0, 1);  Msetv(RC.IKMat, 1, 1,  1);  Msetv(RC.IKMat, 1, 2,  temp);
  Msetv(RC.IKMat, 2, 0, 1);  Msetv(RC.IKMat, 2, 1,  1);  Msetv(RC.IKMat, 2, 2, -temp);
  Msetv(RC.IKMat, 3, 0, 1);  Msetv(RC.IKMat, 3, 1, -1);  Msetv(RC.IKMat, 3, 2,  temp);

  // Parts setup
  SetPartType(0, 1);
  SetPartType(1, 1);
  SetPartType(2, 2);
  SetPartType(3, 3);

  // Machines setup
  machine_pos_x := 0.695 / 2;
  machine_pos_y := 0.08;
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


end;

