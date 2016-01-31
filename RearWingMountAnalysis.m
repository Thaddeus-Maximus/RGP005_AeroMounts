clear

PipeYieldStress = 63100; %Yield stress of 4130 Steel in PSI
PipeModulus = 29700e+3;  %Pipe Modulus of Elasticity in PSI

FOS = 8.5; %Factor of Safety
PipeOD = 0.375; %Pipe OD in inches
syms FrontWallThickness;
PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness*2)^2);

% Attachment Points. Dimensions in Inches.
% Recall coord system: Measured from Car origin. X is forwards, Y is up, Z
% is left
PUpperWing = [-57.80, 39.24, 5.98];
PUpperWing_Left = [PUpperWing(1),PUpperWing(2),-PUpperWing(3)];

PUpperChassis = [-45.36, 39.47, 5.98];
PUpperChassis_Left = [PUpperChassis(1),PUpperChassis(2),-PUpperChassis(3)];


PBottomRearWing = [-65.80,27.24,18.33];
PBottomRearWing_Left = [PBottomRearWing(1),PBottomRearWing(2),-PBottomRearWing(3)];

PBottomRearChassis = [-61.31,14.22,10.86];
PBottomRearChassis_Left = [PBottomRearChassis(1),PBottomRearChassis(2),-PBottomRearChassis(3)];


PBottomFrontWing = [-55.80,27.24,18.33];
PBottomFrontWing_Left = [PBottomFrontWing(1),PBottomFrontWing(2),-PBottomFrontWing(3)];

PBottomFrontChassis = [-60.79,12.71,9.99];
PBottomFrontChassis_Left = [PBottomFrontChassis(1),PBottomFrontChassis(2),-PBottomFrontChassis(3)];

% Tie rod line of actions
LUpper = PUpperChassis - PUpperWing;
LUpper_Left = PUpperChassis_Left - PUpperWing_Left;

LBottomRear = PBottomRearChassis - PBottomRearWing;
LBottomRear_Left = PBottomRearChassis_Left - PBottomRearWing_Left;

LBottomFront = PBottomFrontChassis - PBottomFrontWing;
LBottomFront_Left = PBottomFrontChassis_Left - PBottomFrontWing_Left;

% Tie rod forces
syms TUpper TBottomRear TBottomFront
% TBottomFront=0

FUpper=LUpper/norm(LUpper) *TUpper;
FUpper_Left=LUpper_Left/norm(LUpper_Left) *TUpper;

FBottomRear=LBottomRear/norm(LBottomRear) *TBottomRear;
FBottomRear_Left=LBottomRear_Left/norm(LBottomRear_Left) *TBottomRear;

FBottomFront=LBottomFront/norm(LBottomFront) *TBottomFront;
FBottomFront_Left=LBottomFront_Left/norm(LBottomFront_Left) *TBottomFront;

% Extra Forces in lbf. NEED ESTIMATES
FDown = [0,-500,0];
PDown = [-61, 35, 0];

FDrag = [40,0,0];
PDrag = [-60, 35, 0];

eqs = [FDown + FUpper+FUpper_Left + FBottomRear+FBottomRear_Left + FBottomFront+FBottomFront_Left == [0,0,0],
    cross(PDown, FDown) + cross(PUpperWing, FUpper) + cross(PUpperWing_Left, FUpper_Left) + cross(PBottomRearWing, FBottomRear) + cross(PBottomRearWing_Left, FBottomRear_Left) + cross(PBottomFrontWing, FBottomFront) + cross(PBottomFrontWing_Left, FBottomFront_Left) == [0,0,0] ]
[TBottomRear, TBottomFront, TUpper] = solve(eqs)

display(TBottomRear)


UpperWallThickness_Yield = (solve(PipeYieldStress == abs(TUpper)*FOS/PipeArea));
BottomRearWallThickness_Yield = (solve(PipeYieldStress == abs(TBottomRear)*FOS/PipeArea));
BottomFrontWallThickness_Yield = (solve(PipeYieldStress == abs(TBottomFront)*FOS/PipeArea));
FrontWallThickness = 0.028;
%PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness_Yield(FrontWallThickness<0.1)*2)^2);
%Deformation_Tierod = TFront*norm(LBottomFront)/PipeModulus/PipeArea;

fprintf('Tension in Upper Rods: %f lbs\n', double(TUpper) )
fprintf('Tension in Bottom Rear Rods: %f lbs\n', double(TBottomRear) )
fprintf('Tension in Bottom Front Rods: %f lbs\n', double(TBottomFront) )
fprintf('Required Upper Tie Rod Wall Thickness (Yield, w. FOS): %f0.2 in\n', double(max(UpperWallThickness_Yield(UpperWallThickness_Yield<0.1))) )
fprintf('Required Bottom Rear Tie Rod Wall Thickness (Yield, w. FOS): %f0.2 in\n', double(max(BottomRearWallThickness_Yield(BottomRearWallThickness_Yield<0.1))) )
fprintf('Required Bottom Front Tie Rod Wall Thickness (Yield, w. FOS): %f0.2 in\n', double(max(BottomFrontWallThickness_Yield(BottomFrontWallThickness_Yield<0.1))) )
%fprintf('Deformation at Loading (Realistic, no FOS): %f0.2 in\n\n',  double(Deformation_Tierod) ) 
