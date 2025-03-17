const
 NumJoints = 3;
 a_1 = 0.7;
 a_2 = 0.5;

// Global Variables Here
var
  iZAxis, iJ1Axis, iJ2Axis, iPen: integer;
  tcount: double;


// Place here the Inverse Kinematics calculations
// Return a column matrix with Theta1, Theta2 and Theta3
function IK3(XYZ: matrix): matrix;
var
  xc, yc, zc, c2: double;
  theta1, theta2, disp3: double;
begin
  xc := Mgetv(XYZ, 0, 0);
  yc := Mgetv(XYZ, 1, 0);
  zc := Mgetv(XYZ, 2, 0);

  // Your calculation here

  result := Mzeros(3, 1);
  MSetV(result, 0, 0, theta1);
  MSetV(result, 1, 0, theta2);
  MSetV(result, 2, 0, disp3);
end;



// this procedure is called periodicaly (default: 40 ms)
procedure Control;
var
  t: tcanvas;
  PosPen: TPoint3D;
  // TPoint3D = record
  //   x: double;
  //   y: double;
  //   z: double;
  // end;
begin
  if RCButtonPressed(1, 2) then
    SetSensorColor(0, 0, round(GetRCValue(1, 3)), round(GetRCValue(1, 4)), round(GetRCValue(1, 5)));

  if RCButtonPressed(3, 2) then
    SetAxisPosRef(0, iZAxis, GetRCValue(3, 3));

  if RCButtonPressed(4, 2) then
    SetAxisPosRef(0, iZAxis, GetRCValue(4, 3));

  if RCButtonPressed(6, 2) then begin
    SetAxisPosRef(0, iJ1Axis, GetRCValue(6, 3));
    SetAxisPosRef(0, iJ2Axis, GetRCValue(6, 4));

  end;

  if RCButtonPressed(7, 2) then begin
    SetAxisPosRef(0, iJ1Axis, 0);
    SetAxisPosRef(0, iJ2Axis, 0);
  end;

  PosPen := GetSolidPos(0, iPen);
  SetRCValue(2, 13, format('%.3g',[PosPen.x]));

  t := GetSolidCanvas(0,0);
  t.brush.color := clwhite;
  t.textout(10,10, GetRCText(9, 2));

  tcount := tcount + 0.04;
end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  iJ1Axis := GetAxisIndex(0, 'Joint1', 0);
  iJ2Axis := GetAxisIndex(0, 'Joint2', 0);
  iZAxis := GetAxisIndex(0, 'SlideZ', 0);
  iPen := GetSolidIndex(0, 'Pen');
end;
