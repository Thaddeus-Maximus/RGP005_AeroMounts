clear all

PipeYieldStress = 63100; %Yield stress of 4130 Steel in PSI
PipeModulus = 29700e+3;  %Pipe Modulus of Elasticity in PSI

FOS = 1; %Factor of Safety
PipeOD = 0.375; %Pipe OD in inches
syms FrontWallThickness;
PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness*2)^2);

% Acceleration Info
%TODO: Replace placeholder values
mass = 8/32.2; % Mass in slugs
acceleration = [-19.0*32.2,-32.2,0]; % Acceleration in ft/s^2
PCOM = [22.04, 8, 0]; % Center of mass

% Bottom Attachment Forces
syms FBx FBy;
FBottom=[FBx, FBy, 0];
FBottom_Left=FBottom;

% Attachment Points. Dimensions in Inches. Obtained via SOLIDWORKS
% measuring tool.
% Recall coord system: Measured from Car origin. X is forwards, Y is up, Z
% is left
PFrontWing = [26.31, 5.34, 10];
PFrontWing_Left = [PFrontWing(1),PFrontWing(2),-PFrontWing(3)];

PBottomWing = [12.75,6.57,10];
PBottomWing_Left = [PBottomWing(1),PBottomWing(2),-PBottomWing(3)];

PFrontChassis = [9.75,17.5, 5.5];
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
FDown = [0,-92/2,0];
FDown_Left = [0,-92/2,0];

FDrag = [-12/2, 0, 0];
FDrag_Left = [-12/2, 0, 0];

PAero = [25, 5, 20];
PAero_Left = [PAero(1), PAero(2), -PAero(3)];

% Failure analysis
%fImpact = convforce((120-45)*1000, 'N', 'lbf');
syms fimpact
FImpact = [-fimpact, 0, 0];
PImpact = [29.31, (5.40+1.75)/2, 0];

No8UltTensileStress = 150000; % psi of grade 8 bolt
No8UltShearStress = No8UltTensileStress*0.6; % as per http://www.portlandbolt.com/technical/faqs/calculating-strength/
No8Diam = 0.1585; % inches; minor diameter obtained from http://www.engineershandbook.com/Tables/threadlimits.htm
No8FailForce = pi/4*No8UltShearStress*No8Diam^2; %psi*in^2 = lbf

eq_Failure = No8FailForce == sqrt(FBx^2+ FBy^2);
eq_COLM = FDown+FDown_Left+FDrag+FDrag_Left+FFront + FFront_Left + FBottom + FBottom_Left+FImpact == mass*acceleration;
eq_COAM = cross(PAero, FDown)+cross(PAero_Left, FDown_Left)+cross(PAero, FDrag)+cross(PAero_Left, FDrag_Left)+ cross(PFrontWing, FFront) + cross(PFrontWing_Left, FFront_Left) + cross(PBottomWing, FBottom) + cross(PBottomWing_Left, FBottom_Left) + cross(PImpact,FImpact) == mass*cross(PCOM, acceleration);

% Solve with an initial guess to make sure we get positive impact force
[FBx, FBy, TFront, fimpact] = vpasolve([eq_COLM, eq_COAM, eq_Failure], [FBx, FBy, TFront, fimpact], [4000, 1000, -1400, 6000]);
  
% Shear force is magnitude of x and y forces
FBottomShear = sqrt(FBx^2+FBy^2);

FrontWallThickness_Yield = (solve(PipeYieldStress == TFront*FOS/PipeArea));
FrontWallThickness = 0.028;
PipeArea = pi/4*(PipeOD^2-(PipeOD-FrontWallThickness_Yield(FrontWallThickness<0.1)*2)^2);
Deformation_Tierod = TFront*norm(LFront)/PipeModulus/PipeArea;


fprintf('-- Results for %s: --\n\n', mfilename);
fprintf('Tension in Rod End: %0.2f lbf\n', double(TFront) );
fprintf('Shear Force in Bolt (x,y -> mag): %0.2f, %0.2f -> %0.5f lbf, or %0.5f kN\n', double(FBx), double(FBy), double(FBottomShear), double(FBottomShear*4.44822/1000))
fprintf('Required Tie Rod Wall Thickness (Yield, w. FOS): %0.5f in\n', double(FrontWallThickness_Yield(FrontWallThickness_Yield<0.1)) )
fprintf('Deformation at Loading (Realistic, no FOS): %0.5f in\n',  double(Deformation_Tierod) ) 
fprintf('Impact force required to shear bottom #8 bolts (at %0.0f psi shear): %0.5f lbf, or %0.5f kN\n', No8UltShearStress, double(fimpact), double(fimpact*4.44822/1000) ) % Note: Converting lfb to kN
fprintf('\n-- End of %s --\n\n', mfilename);

%%%% PROGRAM OUTPUT %%%%
% -- Results for FrontWingMountAnalysis: --
% 
% Tension in Rod End: 658.21 lbf
% Shear Force in Bolt (x,y -> mag): 1743.22, -338.55 -> 1775.78721 lbf, or 7.89909 kN
% Required Tie Rod Wall Thickness (Yield, w. FOS): 0.00907 in
% Deformation at Loading (Realistic, no FOS): 0.04468 in
% Impact force required to shear bottom #8 bolts (at 90000 psi shear): 2589.92460 lbf, or 11.52055 kN
% 
% -- End of FrontWingMountAnalysis --