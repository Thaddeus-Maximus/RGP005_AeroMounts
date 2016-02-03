clear

PipeYieldStress = 63100; %Yield stress of 4130 Steel in PSI
PipeModulus = 29700e+3;  %Pipe Modulus of Elasticity in PSI

FOS = 1; %Factor of Safety
PipeOD = 0.375; %Pipe OD in inches
FrontWallThickness = 0.028;
PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness*2)^2);

% Acceleration Info
%TODO: Replace placeholder values
mass = 8/32.2; % Mass in slugs
acceleration = [0,32.2*3,0]; % Acceleration in ft/s^2
PCOM = [-30, 10, 15]; % Center of mass

% Attachment Points. Dimensions in Inches.
% Recall coord system: Measured from Car origin. X is forwards, Y is up, Z
% is left
ZDistWing = 11.8;


PFrontWing = [-21.82, 4.46, ZDistWing];
PFrontChassis = [PFrontWing(1), PFrontWing(2), 8.05];

PMiddleWing = [-26.13, 3.12, ZDistWing];
PMiddleChassis = [PMiddleWing(1), PMiddleWing(2), 8.29];

PRearWing = [-30.49, 5.54, ZDistWing];
PRearChassis = [PRearWing(1), PRearWing(2), 9.47];

% Tie rod line of actions
LFront = PFrontChassis - PFrontWing;
LMiddle = PMiddleChassis - PMiddleWing;
LRear = PRearChassis - PRearWing;

% Tie rod forces
% syms TFront TMiddle TRear;
% FFront=LFront/norm(LFront)*TFront;
% FMiddle=LMiddle/norm(LMiddle)*TMiddle;
% FRear=LRear/norm(LRear)*TRear;

syms FFx FFy FFz FMx FMy FMz FRx FRy FRz
FFront = [FFx, FFy, FFz];
FMiddle = [FMx, FMy, FMz];
FRear = [FRx, FRy, FRz];

% Extra Forces in lbf
FDown = [0,-20,0];
FDrag = [-10, 0, 0];

PAero = [-30, 10, 15];

eq_COLM = FDown+FDrag +FFront+FMiddle+FRear == mass*acceleration;
eq_COAM = cross(PAero,FDown)+cross(PAero,FDrag)+cross(PFrontWing,FFront)+cross(PMiddleWing,FMiddle)+cross(PRearWing,FRear) == mass*cross(PCOM, acceleration);


DefRear = FRear*norm(LRear)/PipeModulus/PipeArea;
DefMiddle = FMiddle*norm(LMiddle)/PipeModulus/PipeArea;
DefFront = FFront*norm(LFront)/PipeModulus/PipeArea;

eq_geo = [norm(DefRear+PRearWing-(DefMiddle+PMiddleWing))==norm(PRearWing-PMiddleWing),...
    norm(DefFront+PFrontWing-(DefMiddle+PMiddleWing))==norm(PFrontWing-PMiddleWing),...
    norm(DefRear+PRearWing-(DefFront+PFrontWing))==norm(PRearWing-PFrontWing)];

[FFx FFy FFz FMx FMy FMz FRx FRy FRz] = solve([eq_COLM, eq_COAM, eq_geo]);

FFront = [FFx, FFy, FFz];
FMiddle = [FMx, FMy, FMz];
FRear = [FRx, FRy, FRz];

fprintf('FFront= [%.2f, %.2f, %.2f]\n', double(FFront));
fprintf('FMiddle= [%.2f, %.2f, %.2f]\n', double(FMiddle));
fprintf('FRear= [%.2f, %.2f, %.2f]\n', double(FRear));
