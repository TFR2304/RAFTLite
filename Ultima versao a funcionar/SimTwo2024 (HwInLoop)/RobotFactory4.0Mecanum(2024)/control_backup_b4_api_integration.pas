type
  T4MecanumRobotControls = record
    iRobot : Integer;
    iMotor1, iMotor2, iMotor3, iMotor4 : Integer;
    pose : TState2D;
    a, b, r : double;
    V, Vn, w : double;
    w1, w2, w3, w4 : double;
    e1, e2, e3, e4 : longInt;
    //frontLeft, frontRight, backLeft, backRight

  end;

  TControlMode = (cmManual, cmAPI, cmMachine);

// Global Variables Here
var

  //Robots
  RC : T4MecanumRobotControls;

  //mecanum
  previousVxy : double;

  //Auxiliary variables
  active : Boolean;
  inactive : Boolean;

  state, iRobot, iLaser : integer;

  start, finish : boolean;

  nextState : boolean;
  reached : boolean;

  currentTarget : TState2D;


  rcKPP, rcKPT, linearSpeed, angularSpeed, poseThreshold, angleThreshold, maxdV : double;

  dt : double;

  log : TStringList;

  ControlMode : TControlMode;

function PrepareLaserSweepPackage(LaserScanRays: Matrix) : string;
var
    i, nrows : integer;
    temp : string;
begin
  //d := (mgetv(LaserScanRays, 0, 0));
  temp := '';
  nrows := MNumRows(LaserScanRays);

  for i := 0 to nrows - 1 do
  begin
    temp := temp + Format('%.4g', [Mgetv(LaserScanRays, i, 0)]) + ',';
  end;


  Delete(temp, Length(temp), 1);
  Result:=temp;


end;

procedure EncodeInteger(var StrPacket: TStringList; name: string; data: integer);
begin
  StrPacket.add(name);
  StrPacket.add(format('%d',[data]));
  StrPacket.add('');
end;

procedure EncodeDouble(var StrPacket: TStringList; name: string; data: double);
begin
  StrPacket.add(name);
  StrPacket.add(format('%.4g',[data]));
  StrPacket.add('');
end;

procedure EncodeDoubleFmt(var StrPacket: TStringList; name: string; data: double; fmt: string);
begin
  StrPacket.add(name);
  StrPacket.add(format(fmt,[data]));
  StrPacket.add('');
end;


function DecodeDoubleDef(var StrPacket: TStringList; name: string; defval: double): double;
var i: integer;
begin
  result := defval;
  i := StrPacket.indexof(name);
  if (i < 0) or (i + 1 >= StrPacket.count) then exit;
  result := strtofloat(StrPacket[i+1]);
end;

procedure updateRobotsStates();
begin

  RC.pose := GetRobotPos2D(RC.iRobot);

  // Mecanum

  SetRCValue(1, 2, Format('%.2g', [RC.V]));
  SetRCValue(2, 2, Format('%.2g', [RC.Vn]));
  SetRCValue(3, 2, Format('%.2g', [RC.w]));

  SetRCValue(4, 2, Format('%.2g', [RC.pose.x]));
  SetRCValue(5, 2, Format('%.2g', [RC.pose.y]));
  SetRCValue(6, 2, Format('%.4g', [deg(RC.pose.angle)]));

  SetRCValue(7, 2, Format('%.4g', [RC.w1]));
  SetRCValue(8, 2, Format('%.4g', [RC.w2]));
  SetRCValue(9, 2, Format('%.4g', [RC.w3]));
  SetRCValue(10, 2, Format('%.4g', [RC.w4]));

end;

function diffAngle(ang1 : double; ang2 : double) : double;
var diff : double;
begin
  diff := ang1 - ang2;

  if (diff < - Pi()) then
  begin

    diff := diff + 2 * Pi();

  end else if (diff > Pi()) then
  begin

    diff := diff - 2 * Pi();

  end;

  result := diff;

end;

procedure mecIK();
var
  processModel, input, output : Matrix;
  temp : double;
begin

  temp := RC.a + RC.b;

  MInit(processModel, 4, 3);
  MInit(input, 3, 1);
  MInit(output, 4, 1);

  Msetv(processModel, 0, 0, 1); Msetv(processModel, 0, 1, -1); Msetv(processModel, 0, 2, -temp);
  Msetv(processModel, 1, 0, 1); Msetv(processModel, 1, 1, 1); Msetv(processModel, 1, 2, temp);
  Msetv(processModel, 2, 0, 1); Msetv(processModel, 2, 1, 1); Msetv(processModel, 2, 2, -temp);
  Msetv(processModel, 3, 0, 1); Msetv(processModel, 3, 1, -1); Msetv(processModel, 3, 2, temp);

  Msetv(input, 0, 0, RC.V);
  Msetv(input, 1, 0, RC.Vn);
  Msetv(input, 2, 0, RC.w);

  output := MmultReal(Mmult(processModel, input), (1 / RC.r));

  RC.w1 := Mgetv(output, 0, 0);
  RC.w2 := Mgetv(output, 1, 0);
  RC.w3 := Mgetv(output, 2, 0);
  RC.w4 := Mgetv(output, 3, 0);

end;

procedure followLine4Mec(x: double; y: double;
theta: double; speed: double);
var
  yNC, thetaNC : double; //robot state in new coordinates
  kY, kTheta: double; //gain cosntants
  vx, vy, vxy : double; // reference speed in world's frame

begin

  // reference speed vector in world's frame
  vxy := speed;

  //representing robot state in new coordinates
  yNC := -(RC.pose.x - x)*sin(theta) + (RC.pose.y - y) * cos(theta);
  thetaNC := diffAngle(RC.pose.angle, theta);
  // setting gains
  kY := 1;

  //computing the follow line equations

  vy := - kY * yNC;

  if abs(vy) >= abs(vxy) then
  begin
    //vy := - speed * (vy / abs(vy));
    //vx := 0;
    vx := 0;
  end else vx := sqrt(power(vxy, 2) - power(vy, 2));

  if speed < 0 then vx := -vx;

  RC.V :=   sin(thetaNC) * vy + cos(thetaNC) * vx;
  RC.Vn := - sin(thetaNC) * vx + cos(thetaNC) * vy;

  // Controlling robot's orientation evolution in world's frame

  kTheta := 4;

  RC.w := - kTheta * thetaNC;

  // visualization

  SetRCValue(3, 2, Format('%.2g', [yNC]));
  SetRCValue(4, 2, Format('%.4g', [deg(thetaNC)]));

  // Target

  SetRCValue(13, 4, Format('%.2g', [x]));
  SetRCValue(14, 4, Format('%.2g', [y]));
  SetRCValue(15, 4, Format('%.4g', [deg(theta)]));
  SetRCValue(17, 4, Format('%.2g', [speed]));

end;

procedure followCircle4Mec(x: double; y: double;
radius: double; speed: double);
var
  yNC, thetaNC, theta : double; //robot state in new coordinates
  kY, kTheta: double; //gain cosntants
  vx, vy, vxy : double; // reference speed in world's frame
begin

  vxy := speed;
  //representing robot state in new coordinates
  //xNC := (theRobotState.x - x)*cos(theta) + (theRobotState.y - y) * sin(theta);
  yNC := sqrt(sqr(RC.pose.x - x) + sqr(RC.pose.y - y)) - radius;
  theta := atan2(RC.pose.y - y, RC.pose.x - x) - pi / 2;
  thetaNC := diffAngle(RC.pose.angle, theta);


  // trajectory control
  ky := 15;

  vy := - kY * yNC;

  if abs(vy) >= abs(vxy) then
  begin

    vx := 0;

  end else vx := sqrt(power(vxy,2) - power(vy, 2));

  if speed < 0 then vx := -vx;

  RC.V :=   sin(thetaNC) * vy + cos(thetaNC) * vx;
  RC.Vn := - sin(thetaNC) * vx + cos(thetaNC) * vy;



  // orientation control
  kTheta := 0; //4.5

  RC.w := - kTheta * thetaNC;


  //visualization

  SetRCValue(6, 2, Format('%.2g', [yNC]));
  SetRCValue(7, 2, Format('%.4g', [deg(theta)]));
  SetRCValue(8, 2, Format('%.4g', [deg(thetaNC)]));

  SetRCValue(13, 4, Format('%.2g', [x]));
  SetRCValue(14, 4, Format('%.2g', [y]));
  SetRCValue(15, 4, Format('%.4g', [deg(theta)]));
  SetRCValue(16, 4, Format('%.2g', [radius]));
  SetRCValue(17, 4, Format('%.2g', [speed]));

end;

function accelConstraint(dV : double; maxdV : double; mindV : double) : double;
begin

  if (dV >= maxdV) then
  begin
    dV := maxdV;
  end else if (dV <= mindV) then
  begin
    dV := mindV;
  end;

  result := dV;

end;

function min(a : double; b : double) : double;
begin

  if (a < b) then
  begin
    result := a;
  end
  else begin
   result := b;
   end;

end;

function posThreshold(RC : TState2D; target : TState2D; threshold : double) : boolean;
var
  ePosition : double;
begin

  result := False;

  ePosition := sqrt(sqr(RC.x - target.x) + sqr(RC.y - target.y));

  if ePosition <= threshold then result := True;

end;

function rotThreshold(RC : double; target : double; threshold : double) : boolean;
var
  eAngle : double;
begin

  result := False;

  eAngle := DiffAngle(target, RC);

  if abs(eAngle) <= threshold then result := True;

end;

procedure gotoXY4Mec(x : double; y : double;
speed : double; rAngle : double; aSpeed : double; Kpp : double; Kpt : double);
var
  vxy, vx, vy, hAngle, dist: double;
  deltaX, deltaY, eAngle, kTheta : double;
  dV : double;
  abs_w, abs_speed, min_abs : double;
begin
  deltaX := x - RC.pose.x;
  deltaY := y - RC.pose.y;

  dist := sqrt(sqr(deltaX) + sqr(deltaY));

  vxy := Kpp * dist;

  if vxy > speed then vxy := speed;

  dV := vxy - previousVxy;

  dV := accelConstraint(dV, maxdV, -maxdV);

  previousVxy := previousVxy + dV;

  vxy := previousVxy;

  hAngle := atan2(deltaY, deltaX);

  vx := vxy * cos(hAngle);
  vy := vxy * sin(hAngle);

  RC.V := vx * cos(-RC.pose.angle) - vy * sin(-RC.pose.angle);
  RC.Vn := vx * sin(-RC.pose.angle) + vy * cos(-RC.pose.angle);

  //Orientation control

  eAngle := DiffAngle(rAngle, RC.pose.Angle);

  kTheta := Kpt;

  RC.w := kTheta * eAngle;

  if (RC.w < 0) then aSpeed := - aSpeed;

  abs_w := abs(RC.w);
  abs_speed := abs(aSpeed);
  min_abs := min(abs_w, abs_speed);

  if (RC.w < 0) then
  begin
    RC.w := -min_abs;
  end
  else
  begin
    RC.w := min_abs;
  end;

  // visualization

  SetRCValue(13, 2, Format('%.2g', [dist]));
  SetRCValue(14, 2, Format('%.4g', [deg(vxy)]));
  SetRCValue(15, 2, Format('%.4g', [deg(hAngle)]));

  SetRCValue(20, 2, Format('%.2g', [x]));
  SetRCValue(21, 2, Format('%.2g', [y]));
  SetRCValue(22, 2, Format('%.2g', [speed]));

end;

procedure rotateMec(Kpt: double;
 rAngle : double; aSpeed : double);
var
  eAngle: double;
  abs_w , abs_speed, min_abs : double;
begin

  eAngle := DiffAngle(rAngle, RC.pose.angle);

  RC.V := 0;
  RC.Vn := 0;

  RC.w := Kpt * eAngle;

  if (RC.w < 0) then aSpeed := - aSpeed;

  abs_w := abs(RC.w);
  abs_speed := abs(aSpeed);
  min_abs := min(abs_w, abs_speed);

  if (RC.w < 0) then
  begin
    RC.w := -min_abs;
  end
  else
  begin
    RC.w := min_abs;
  end;

end;



procedure resetRobot();
begin

  SetRobotPos(RC.iRobot, -0.695, -0.355, 0.01, rad(90));
  dt := 0;

  finish := False;
  reached := False;

  log.clear;

end;



procedure stateMachine();
begin

  if (state = 0) and (nextState) then
  begin
    currentTarget.x := -0.555;
    currentTarget.y := 0.245;
    currentTarget.angle := rad(90) - 0.03;

    state := 1;
    nextState := False;
    reached := False;
    start := True;


  end else if (state = 1) and (nextState) then
  begin
    currentTarget.x := 0;
    currentTarget.y := 0.230;
    currentTarget.angle := rad(90) - 0.03;

    state := 2;
    nextState := False;
    reached := False;
  end else if (state = 2) and (nextState) then
  begin
    currentTarget.x := 0;
    currentTarget.y := 0;
    currentTarget.angle := rad(-90);

    state := 3;
    nextState := False;
    reached := False;
  end else if (state = 3) and (nextState) then
  begin
    currentTarget.x := 0;
    currentTarget.y := -0.267;
    currentTarget.angle := rad(-90);

    state := 4;
    nextState := False;
    reached := False;
  end else if (state = 4) and (nextState) then
  begin
    currentTarget.x := 0.265;
    currentTarget.y := -0.244;
    currentTarget.angle := rad(-90);

    state := 5;
    nextState := False;
    reached := False;
  end else if (state = 5) and (nextState) then
  begin
    nextState := False;
    reached := False;
    start := False;
    Finish := True;
  end;


  if (state = 0) then
  begin

    if RCButtonPressed(4, 6) then nextState := True;
    resetRobot();

  end else if (state = 1) then
  begin

    gotoXY4Mec(currentTarget.x, currentTarget.y, linearSpeed, currentTarget.angle, angularSpeed, rcKPP, rcKPT);

    reached := posThreshold(RC.pose, currentTarget, poseThreshold);

    if reached then nextState := True;

  end else if (state = 2) then
  begin

    gotoXY4Mec(currentTarget.x, currentTarget.y, linearSpeed, currentTarget.angle, angularSpeed, rcKPP, rcKPT);

    reached := posThreshold(RC.pose, currentTarget, poseThreshold);

    if reached then nextState := True;

  end else if (state = 3) then
  begin

    rotateMec(rcKPT, currentTarget.angle, angularSpeed);
    reached := rotThreshold(RC.pose.angle, currentTarget.angle, angleThreshold);

    if reached then nextState := True;

  end else if (state = 4) then
  begin

    gotoXY4Mec(currentTarget.x, currentTarget.y, linearSpeed, currentTarget.angle, angularSpeed, rcKPP, rcKPT);

    reached := posThreshold(RC.pose, currentTarget, poseThreshold);

    if reached then nextState := True;

  end else if (state = 5) then
  begin

    gotoXY4Mec(currentTarget.x, currentTarget.y, linearSpeed, currentTarget.angle, angularSpeed, rcKPP, rcKPT);

    reached := posThreshold(RC.pose, currentTarget, poseThreshold);

    if reached then nextState := True;

  end;

  SetRCValue(6, 7, IntToStr(state));

end;

procedure actuateRobot();
begin

  mecIK();

  SetAxisSpeedRef(RC.iRobot, RC.iMotor1, RC.w1);
  SetAxisSpeedRef(RC.iRobot, RC.iMotor2, RC.w2);
  SetAxisSpeedRef(RC.iRobot, RC.iMotor3, RC.w3);
  SetAxisSpeedRef(RC.iRobot, RC.iMotor4, RC.w4);

end;

procedure mecanumControl();
begin

  stateMachine();

end;

procedure logDataToCSV();
begin

  log.Add(FloatToStr(RC.pose.x) + ',' + FloatToStr(RC.pose.y) + ',' +
  FloatToStr(RC.pose.angle) + ',' + FloatToStr(dt));

end;

procedure saveToFile(const FileName : String);
begin
    log.SaveToFile(FileName);
    log.Free;
end;

procedure prepareEncodersMsg(var StrPacket : TStringList);
begin

  // Encoders

  RC.e1 := GetAxisOdo(0, 0);
  RC.e2 := GetAxisOdo(0, 1);
  RC.e3 := GetAxisOdo(0, 2);
  RC.e4 := GetAxisOdo(0, 3);

  EncodeDouble(StrPacket, 'EncFL', RC.e1);
  EncodeDouble(StrPacket, 'EncFR', RC.e2);
  EncodeDouble(StrPacket, 'EncBL', RC.e3);
  EncodeDouble(StrPacket, 'EncBR', RC.e4);

end;

procedure prepareLaserMsg(var StrPacket : TStringList);
var
  LaserValues : Matrix;
begin
  LaserValues := GetSensorValues(RC.iRobot, iLaser);
  StrPacket.add('lidar');
  StrPacket.add(PrepareLaserSweepPackage(LaserValues));
  StrPacket.add('');
end;

procedure prepareGroundTruthMsg(var StrPacket : TStringList);
begin

  EncodeDouble(StrPacket, 'X', RC.pose.x);
  EncodeDouble(StrPacket, 'Y', RC.pose.y);
  EncodeDouble(StrPacket, 'THETA', RC.pose.angle);

end;

procedure API();
var
  StrPacket : TStringList;
begin

  try

    StrPacket := TStringList.create;

    // Encoders

    prepareEncodersMsg(StrPacket);

    // Ground Truth

    prepareGroundTruthMsg(StrPacket);

    // LIDAR

    if (iLaser >= 0) then prepareLaserMsg(StrPacket);

    WriteUDPData('127.0.0.1', 9000, StrPacket.text);

    StrPacket.clear;
    StrPacket.text := ReadUDPData();







    // Read Motor Speed Reference
    if StrPacket.text <> '' then
    begin
      RC.w1 := DecodeDoubleDef(StrPacket, 'wFL', 0);
      RC.w2 := DecodeDoubleDef(StrPacket, 'wFR', 0);
      RC.w3 := DecodeDoubleDef(StrPacket, 'wBL', 0);
      RC.w4 := DecodeDoubleDef(StrPacket, 'wBR', 0);
    end;

  finally
    StrPacket.free;
  end;


end;

procedure Manual();
begin
    // Initialize to default (not pressed) values
    RC.V := 0;
    RC.Vn := 0;
    RC.w := 0;

    // Check each key press independently
    if KeyPressed(vk_down) then
        RC.V := -1;

    if KeyPressed(vk_left) then
        RC.Vn := 1;

    if KeyPressed(vk_right) then
        RC.Vn := -1;

    if KeyPressed(vk_up) then
        RC.V := 1;

    if KeyPressed(79) then  // 'O'
        RC.w := 1;

    if KeyPressed(80) then  // 'P'
        RC.w := -1;

end;

// this procedure is called periodicaly (default: 25 ms)
procedure Control;
var StrPacket: TStringList;
    txt: string;
    temp : TState2D;
begin

  //Update robot's states

  updateRobotsStates();

  //Control mode selection

  if (RCButtonPressed(1,6)) then
  begin
    SetRCValue(2, 7, 'Manual');
    ControlMode := cmManual;
  end else if (RCButtonPressed(1,7)) then
  begin
    SetRCValue(2, 7, 'API');
    ControlMode := cmAPI;
  end else if (RCButtonPressed(1,8)) then
  begin
    SetRCValue(2, 7, 'State Machine');
    ControlMode := cmMachine;
  end;


  // High-level control

  case ControlMode of
    cmManual : Manual();
    cmAPI : API();
    cmMachine : mecanumControl();
  end;


  // Low-level control

  actuateRobot();

  //Logging

  if start then
  begin
    logDataToCSV();
    dt := dt + 0.025;
  end;

  if finish then
  begin
    saveToFile('trajectory.txt');
    finish := False;
  end;

end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  iLaser := GetSensorIndex(iRobot, 'ranger2d');

  ControlMode := cmManual;
  SetRCValue(2,7, 'Manual');

  previousVxy := 0;

  active := True;
  inactive := False;
  state := 0;

  start := False;
  finish := False;

  nextState := False;
  reached := False;
  rcKPP := 4;
  rcKPT := 10;
  linearSpeed := 0.5;
  angularSpeed := 2;

  maxdV := 0.025;

  angleThreshold := rad(2.5);
  poseThreshold := 0.02;

  dt := 0;

  log := TStringList.create;

  log.Add('x,y,theta,time');


   // 4 Wheeled Mecanum

  RC.iRobot := GetRobotIndex('mecanum');
  SetRobotPos(RC.iRobot, -0.695, -0.355, 0.01, rad(90));
  RC.iMotor1 := GetAxisIndex(RC.iRobot, 'LeftAxisFront', 0);
  RC.iMotor2 := GetAxisIndex(RC.iRobot, 'RightAxisFront', 0);
  RC.iMotor3 := GetAxisIndex(RC.iRobot, 'LeftAxisBack', 0);
  RC.iMotor4 := GetAxisIndex(RC.iRobot, 'RightAxisBack', 0);

  RC.a := 0.25 / 2 - 0.05 ;
  RC.b := 0.155 / 2 + 0.015 ;

  RC.r := 0.065;

  SetMotorControllerState(RC.iRobot, RC.iMotor1, active);
  SetMotorControllerState(RC.iRobot, RC.iMotor2, active);
  SetMotorControllerState(RC.iRobot, RC.iMotor3, active);
  SetMotorControllerState(RC.iRobot, RC.iMotor4, active);
  SetMotorControllerMode(RC.iRobot, RC.iMotor1, 'pidspeed');
  SetMotorControllerMode(RC.iRobot, RC.iMotor2, 'pidspeed');
  SetMotorControllerMode(RC.iRobot, RC.iMotor3, 'pidspeed');
  SetMotorControllerMode(RC.iRobot, RC.iMotor4, 'pidspeed');

end;

