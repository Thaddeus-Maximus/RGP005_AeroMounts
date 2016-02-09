clear
fprintf('...working...\n');
PipeYieldStress = 63100; %Yield stress of 4130 Steel in PSI
PipeModulus = 29700e+3;  %Pipe Modulus of Elasticity in PSI

% Acceleration Info
%TODO: Replace placeholder values
mass = 20/32.2; % Mass in slugs
acceleration = [32.2,32.2,0]; % Acceleration in ft/s^2
PCOM = [-64, 40, 0]; % Center of mass

FOS = 8.5; %Factor of Safety
PipeOD = 0.375; %Pipe OD in inches
syms WallThickness;
PipeArea = pi/4*(PipeOD^2-(PipeOD-WallThickness*2)^2);

% Attachment Points. Dimensions in Inches.
% Recall coord system: Measured from Car origin. X is forwards, Y is up, Z
% is right
PUpperWing = [-57.80, 39.24, 5.98];
PUpperWing_Left = [PUpperWing(1),PUpperWing(2),-PUpperWing(3)];


PBottomWing = [-65.80,27.24,18.33];
PBottomWing_Left = [PBottomWing(1),PBottomWing(2),-PBottomWing(3)];

PBottomChassis = [-61.31,14.22,10.86];
PBottomChassis_Left = [PBottomChassis(1),PBottomChassis(2),-PBottomChassis(3)];

% Tie rod line of actions
LBottom = PBottomChassis - PBottomWing;
LBottom_Left = PBottomChassis_Left - PBottomWing_Left;


% Mount forces
syms TBottom
syms FUpperx FUppery Fupperz

FUpper = [FUpperx, FUppery, Fupperz];
FUpper_Left = [FUpperx, FUppery, Fupperz];

FBottom=LBottom/norm(LBottom) *TBottom;
FBottom_Left=LBottom_Left/norm(LBottom_Left) *TBottom;

% Extra Forces in lbf.
%TODO: NEED ESTIMATES
FDown = [0,-500,0];
PDown = [-61, 35, 0];

FDrag = [40,0,0];
PDrag = [-60, 35, 0];

FWeight = [0,-mass*32.2,0]; % Weight of wing

eq_COLM = FDown + FUpper+FUpper_Left + FBottom+FBottom_Left + FWeight== mass*acceleration;
eq_COAM = cross(PDown, FDown) + ...
    cross(PUpperWing, FUpper) + cross(PUpperWing_Left, FUpper_Left) + ...
    cross(PBottomWing, FBottom) + cross(PBottomWing_Left, FBottom_Left) + ...
    cross(PCOM,FWeight)== mass*cross(PCOM, acceleration);

[FUpperx, FUppery, FUpperz, TBottom] = solve([eq_COLM, eq_COAM]);
FUpper = [FUpperx, FUppery, FUpperz];

PipeWallThickness = 0.028;
%PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness_Yield(FrontWallThickness<0.1)*2)^2);
%Deformation_Tierod = TFront*norm(LBottomFront)/PipeModulus/PipeArea;

FBottom=LBottom/norm(LBottom) *TBottom;

fprintf('\n');
fprintf('Cartesian force Applied By Each Upper Mount:\n    [%.2f, %.2f, %.2f] lbs\n', double(FUpper) )
fprintf('\n');
fprintf('Tension in Bottom Rear Rods:\n    %.2f lbs\n', double(TBottom) )
fprintf('Cartesian force seen by a tab for Bottom Rear Rods:\n    [%.2f, %.2f, %.2f] lbs\n', double(-FBottom/2) )