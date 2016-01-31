clear

PipeYieldStress = 63100; %Yield stress of 4130 Steel in PSI
PipeModulus = 29700e+3;  %Pipe Modulus of Elasticity in PSI

FOS = 8.5; %Factor of Safety
PipeOD = 0.375; %Pipe OD in inches
syms FrontWallThickness;
PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness*2)^2);


% Bottom Attachment Forces
syms FBx FBy;
FBottom=[FBx, FBy, 0];
FBottom_Left=FBottom;

% Attachment Points. Dimensions in Inches.
% Recall coord system: Measured from Car origin. X is forwards, Y is up, Z
% is left
PFrontWing = [26.31, 5.34, -9.75];
PFrontWing_Left = [PFrontWing(1),PFrontWing(2),-PFrontWing(3)];

PBottomWing = [12.75,6.57,-10];
PBottomWing_Left = [PBottomWing(1),PBottomWing(2),-PBottomWing(3)];

PFrontChassis = [9.75,17.5, -5.5];
PFrontChassis_Left = [PFrontChassis(1),PFrontChassis(2),-PFrontChassis(3)];

% Tie rod line of actions
LFront = PFrontChassis - PFrontWing;
LFront_Left = PFrontChassis_Left - PFrontWing_Left;

% Tie rod forces
syms TFront TFront_Left;
TFront_Left = TFront;
FFront=LFront/norm(LFront)*TFront;
FFront_Left=LFront_Left/norm(LFront_Left)*TFront_Left;

% Extra Forces in lbf
FDown = [0,-200,0];
PDown = [25, 0, 0];

[FBx, FBy, TFront] = solve([
    FDown + FFront + FFront_Left + FBottom + FBottom_Left == [0,0,0],
    cross(PDown, FDown) + cross(PFrontWing, FFront) + cross(PFrontWing_Left, FFront_Left) + cross(PBottomWing, FBottom) + cross(PBottomWing_Left, FBottom_Left) == [0,0,0]
    ]);

% Shear force is magnitude of x and y forces
FBottomShear = sqrt(FBx^2+FBy^2);

FrontWallThickness_Yield = (solve(PipeYieldStress == TFront*FOS/PipeArea));
FrontWallThickness = 0.028
PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness_Yield(FrontWallThickness<0.1)*2)^2);
Deformation_Tierod = TFront*norm(LFront)/PipeModulus/PipeArea;

fprintf('Tension in Rod End: %f0.2 lbs\n', double(TFront) )
fprintf('Shear Force in Bolt (x,y: mag): %f0.2, %f0.2 :%f0.2 lbs\n', double(FBx), double(FBy), double(FBottomShear) )
fprintf('Required Tie Rod Wall Thickness (Yield, w. FOS): %f0.2 in\n', double(FrontWallThickness_Yield(FrontWallThickness_Yield<0.1)) )
fprintf('Deformation at Loading (Realistic, no FOS): %f0.2 in\n\n',  double(Deformation_Tierod) ) 
