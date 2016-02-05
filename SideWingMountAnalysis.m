clear

PipeYieldStress = 63100; %Yield stress of 4130 Steel in PSI
PipeModulus = 29700e+3;  %Pipe Modulus of Elasticity in PSI

FOS = 1; %Factor of Safety
PipeOD = 0.375; %Pipe OD in inches
FrontWallThickness = 0.028;
PipeID = (PipeOD-FrontWallThickness*2);
PipeArea = pi/4*(PipeOD^2-PipeID^2);

% Acceleration Info
%TODO: Replace placeholder values
mass = 8/32.2; % Mass in slugs % Get real
acceleration = [32.2,32.2*3,0]; % Acceleration in ft/s^2
PCOM = [-30, 10, 22]; % Center of mass % Get real

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
FDown = [0,-50,0];   % Get real
FDrag = [-20, 0, 0]; % Get real

PAero = [-28.5, 10, 22]; % Get real

eq_COLM = FDown+FDrag +FFront+FMiddle+FRear == mass*acceleration;
eq_COAM = cross(PAero,FDown)+cross(PAero,FDrag)+cross(PFrontWing,FFront)+cross(PMiddleWing,FMiddle)+cross(PRearWing,FRear) == mass*cross(PCOM, acceleration);

F = sym('F',[1,3]);
L = sym('L',[1,3]);
Def = norm(L)^3*[F(1), F(2), 0]/(3*PipeModulus*(pi*(PipeOD^2-PipeID^4))/64) +[0,0,F(3)*norm(L)/PipeModulus/PipeArea];
DefRear = subs(Def, [F, L], [FRear, LRear]);
DefMiddle = subs(Def, [F, L], [FMiddle, LMiddle]);
DefFront = subs(Def, [F, L], [FFront, LFront]);

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
