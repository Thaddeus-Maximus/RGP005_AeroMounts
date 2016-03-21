clear

fprintf('...working...\n');

PipeYieldStress = 63100; %Yield stress of 4130 Steel in PSI
PipeModulus = 29700e+3;  %Pipe Modulus of Elasticity in PSI

% Acceleration Info
%TODO: Replace placeholder values
mass = 20/32.2;                 % Mass in slugs
acceleration = [-32.2*3,32.2*3,32.2*1.5]; % Acceleration in ft/s^2
PCOM = [-64, 40, 0];            % Center of mass

PipeOD = 0.375;                 %Pipe OD in inches
syms WallThickness;
PipeArea = pi/4*(PipeOD^2-(PipeOD-WallThickness*2)^2);

% Attachment Points. Dimensions in Inches.
% Recall coord system: Measured from Car origin. X is forwards, Y is up, Z
% is right
PUpperWing          = [-62.2, 37.4, 5.79];
PUpperWing_Left     = [PUpperWing(1),PUpperWing(2),-PUpperWing(3)];


PBottomWing         = [-65.80, 27.24, 18.33];
PBottomWing_Left    = [PBottomWing(1),PBottomWing(2),-PBottomWing(3)];

PBottomChassis      = [-61.31, 14.22, 10.86];
PBottomChassis_Left = [PBottomChassis(1),PBottomChassis(2),-PBottomChassis(3)];

% Tie rod line of actions
LBottom = PBottomChassis - PBottomWing;
LBottom_Left = PBottomChassis_Left - PBottomWing_Left;


% Mount forces
syms TBottom TBottom_Left
syms FUpperx FUppery FUpperz
syms FUpper_Leftx

FUpper = [FUpperx, FUppery, FUpperz];
FUpper_Left = [FUpper_Leftx, FUppery, FUpperz]; % Assume the unit acts rigidly up and down and side to side, but not forwards-backwards

FBottom=LBottom/norm(LBottom) *TBottom;
FBottom_Left=LBottom_Left/norm(LBottom_Left) *TBottom_Left;

% Extra Forces in lbf.
%TODO: NEED ESTIMATES
FDown = [0,-500,0];
PDown = [-64, 35, 0];

FDrag = [-40,0,0];
PDrag = [-64, 35, 0];

eq_COLM_wing = FDown+FDrag+ FUpper+FUpper_Left + FBottom+FBottom_Left == mass*acceleration;
eq_COAM_wing = cross(PDown, FDown)+cross(PDrag,FDrag) + ...
    cross(PUpperWing, FUpper) + cross(PUpperWing_Left, FUpper_Left) + ...
    cross(PBottomWing, FBottom) + cross(PBottomWing_Left, FBottom_Left) == mass*cross(PCOM, acceleration);

[FUpperx, FUppery, FUpperz, FUpper_Leftx, TBottom, TBottom_Left] = solve([eq_COLM_wing, eq_COAM_wing],[FUpperx, FUppery, FUpperz, FUpper_Leftx, TBottom, TBottom_Left]);

FUpper = [FUpperx, FUppery, FUpperz];
FUpper_Left = [FUpper_Leftx, FUppery, FUpperz];

PipeWallThickness = 0.028;
%PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness_Yield(FrontWallThickness<0.1)*2)^2);
%Deformation_Tierod = TFront*norm(LBottomFront)/PipeModulus/PipeArea;

FBottom=LBottom/norm(LBottom) *TBottom;


%Truss Analysis

FUpperxy = [FUpperx, FUppery];

syms FTrussUpperBoltx FTrussUpperBolty
syms FTrussBottomBoltx FTrussBottomBolty
%FTrussBottomBolt = sym('FTrussBottomBolt', [1 3]);
FTrussBottomBolt = [FTrussBottomBoltx FTrussBottomBolty 0];
FTrussUpperBolt = [FTrussUpperBoltx FTrussUpperBolty 0];

%FTrussUpperBolt = sym('FTrussUpperBolt', [1 3]);

PTrussBottomBolt = [-46.16233, 37.0,PUpperWing(3)];
PTrussUpperBolt =  [-44.75,41.75,PUpperWing(3)];

eq_COLM_truss = -FUpper + FTrussBottomBolt + FTrussUpperBolt == [0,0,0];
eq_COAM_truss = cross(PUpperWing,-FUpper) + cross(PTrussBottomBolt,FTrussBottomBolt) + cross(PTrussUpperBolt,FTrussUpperBolt) == [0,0,0];
[FTrussUpperBoltx, FTrussUpperBolty, FTrussBottomBoltx, FTrussBottomBolty] = solve([eq_COLM_truss, eq_COAM_truss],[FTrussUpperBoltx, FTrussUpperBolty, FTrussBottomBoltx, FTrussBottomBolty]);

FTrussBottomBolt = [FTrussBottomBoltx FTrussBottomBolty 0];
FTrussUpperBolt = [FTrussUpperBoltx FTrussUpperBolty 0];

fprintf('\n');
fprintf('Cartesian force Applied By Each Upper Mount:\n    Right: [%.2f, %.2f, %.2f] lbs\n    Left:  [%.2f, %.2f, %.2f] lbs\n', double(FUpper), double(FUpper_Left) )
fprintf('\n');
fprintf('Tension in Bottom Rear Rods:\n    Right: %.2f lbs\n    Left:  %.2f lbs\n', double(TBottom), double(TBottom_Left) )
fprintf('Cartesian force seen by a tab for Bottom Rear Rods:\n    [%.2f, %.2f, %.2f] lbs\n', double(-FBottom/2) )
fprintf('\n');
fprintf('Force provided by bolts in mount:\n    Bottom: [%.2f, %.2f, %.2f]; |%.2f| lbs\n       Top: [%.2f, %.2f, %.2f]; |%.2f| lbs\n', double(FTrussBottomBolt), double(norm(FTrussBottomBolt)), double(FTrussUpperBolt), double(norm(FTrussUpperBolt)));