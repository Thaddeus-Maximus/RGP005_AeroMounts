clear all

PipeYieldStress = 63100; %Yield stress of 4130 Steel in PSI
PipeModulus = 29700e+3;  %Pipe Modulus of Elasticity in PSI

FOS = 1; %Factor of Safety
PipeOD = 0.375; %Pipe OD in inches
FrontWallThickness = 0.028;
PipeID = (PipeOD-FrontWallThickness*2);
PipeArea = pi/4*(PipeOD^2-PipeID^2);

% Acceleration Info
%TODO: Replace placeholder values
mass = 6/32.2; % Mass in slugs
acceleration = [32.2,32.2,-32.2]*3; % Acceleration in ft/s^2
PCOM = [-33.4, 8.5, 20]; % Center of mass

% Attachment Points. Dimensions in Inches.
% Recall coord system: Measured from Car origin. X is forwards, Y is up, Z
% is left
ZDistWing = 12.5;


PFrontWing = [-25.53, 4.43, ZDistWing];
PFrontChassis = [PFrontWing(1), PFrontWing(2), 8.05];

PMiddleWing = [-29.15, 3.06, ZDistWing];
PMiddleChassis = [PMiddleWing(1), PMiddleWing(2), 8.29];

PRearWing = [-41.73,11.72,13.42];
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
FDown = [0,-45,0];
FDrag = [-15, 0, 0];

PAero = [-33.4, 8.5, 20];

eq_COLM = FDown+FDrag +FFront+FMiddle+FRear == mass*acceleration;
eq_COAM = cross(PAero,FDown)+cross(PAero,FDrag)+cross(PFrontWing,FFront)+cross(PMiddleWing,FMiddle)+cross(PRearWing,FRear) == mass*cross(PCOM, acceleration);

F = sym('F',[1,3]);
L = sym('L',[1,3]);
Def = norm(L)^3*[F(1), F(2), 0]/(3*PipeModulus*(pi*(PipeOD^2-PipeID^4))/64) +[0,0,F(3)*norm(L)/PipeModulus/PipeArea];
DefRear = subs(Def, [F, L], [FRear, LRear]);
DefMiddle = subs(Def, [F, L], [FMiddle, LMiddle]);
DefFront = subs(Def, [F, L], [FFront, LFront]);

% Distance between mount points should remain roughly the same; use this as
% a constraint
eq_geo = [norm(DefRear+PRearWing-(DefMiddle+PMiddleWing))==norm(PRearWing-PMiddleWing),...
    norm(DefFront+PFrontWing-(DefMiddle+PMiddleWing))==norm(PFrontWing-PMiddleWing),...
    norm(DefRear+PRearWing-(DefFront+PFrontWing))==norm(PRearWing-PFrontWing)];

[FFx FFy FFz FMx FMy FMz FRx FRy FRz] = vpasolve([eq_COLM, eq_COAM, eq_geo]);

FFront = [FFx, FFy, FFz];
FMiddle = [FMx, FMy, FMz];
FRear = [FRx, FRy, FRz];

DefRear = subs(Def, [F, L], [FRear, LRear]);
DefMiddle = subs(Def, [F, L], [FMiddle, LMiddle]);
DefFront = subs(Def, [F, L], [FFront, LFront]);

fprintf('FFront=  [%.2f, %.2f, %.2f]\n', double(FFront));
fprintf('FMiddle= [%.2f, %.2f, %.2f]\n', double(FMiddle));
fprintf('FRear=   [%.2f, %.2f, %.2f]\n', double(FRear));
fprintf('\n');
fprintf('DefFront_wingtip=  [%.4f, %.4f] %.0f\n', double(DefFront/norm(LFront)*19));
fprintf('DefMiddle_wingtip= [%.4f, %.4f] %.0f\n', double(DefMiddle/norm(LMiddle)*19));
fprintf('DefRear_wingtip=   [%.4f, %.4f] %.0f\n', double(DefRear/norm(LRear)*19));
fprintf('\n');
fprintf('FYield= %.2f\n', PipeYieldStress*PipeArea)
fprintf('FYield for threads in aluminum= %.2f\n', 30000 *pi*0.25*0.25/2)
