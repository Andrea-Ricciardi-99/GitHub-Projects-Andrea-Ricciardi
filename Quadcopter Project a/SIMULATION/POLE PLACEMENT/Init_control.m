%%%  AttitudeControl_tune
%clear
path('./icon/',path);
Init;

% constant value
RAD2DEG = 57.2957795;
DEG2RAD = 0.0174533;
% throttle when UAV is hovering
THR_HOVER = 0.609;


R_p_x=sqrt(2)/2*[-1 1;
                 -1 -1];

%% Initial condition
ModelInit_PosE = [0, 0, -100];
ModelInit_VelB = [0, 0, 0];
ModelInit_AngEuler = [0.4, 0.4, 0.2];
ModelInit_RateB = [0, 0, 0];
ModelInit_Rads = 557.142;
%% control parameter
% attitude PID parameters
Kp_PITCH_ANGLE = 6.5;
Kp_PITCH_AngleRate = 0.1;
Ki_PITCH_AngleRate = 0.02;
Kd_PITCH_AngleRate = 0.001;
Kp_ROLL_ANGLE = 6.5;
Kp_ROLL_AngleRate = 0.1;
Ki_ROLL_AngleRate = 0.02;
Kd_ROLL_AngleRate = 0.001;

Kp_YAW_AngleRate = 0.3;
Ki_YAW_AngleRate = 0.1;
Kd_YAW_AngleRate = 0.00;
% integral saturation
Saturation_I_RP_Max = 0.3;
Saturation_I_RP_Min = -0.3;
Saturation_I_Y_Max = 0.2;
Saturation_I_Y_Min = -0.2;
% max control angle, default 35-deg
MAX_CONTROL_ANGLE_ROLL = 35;
MAX_CONTROL_ANGLE_PITCH  = 35;
% max control angle rate, rad/s 
MAX_CONTROL_ANGLE_RATE_PITCH = 220;
MAX_CONTROL_ANGLE_RATE_ROLL = 220;
MAX_CONTROL_ANGLE_RATE_Y = 200;
%% State space representation
m = 1.07;
L = 0.25;
d = 0.02;
J = 0.055;
g = 9.81;
Ts = 0.002;
T1 = 3;
T2 = 3;
T3 = 3;
T4 = 3;

A = [0 1 0 0;
     (m*g*d-T1*d-T2*d)/J 0 0 0;
     0 0 0 1;
     0 0 (m*g*d-T3*d-T4*d)/J 0];
B = [ 0 0 0 0;
      L/J -L/J 0 0;
      0 0 0 0;
      0 0 -L/J L/J];
C = sqrt(2)/2*[0 -1 0 1;
               0 -1 0 -1];
D = zeros(2,4);
dsys = c2d(ss(A,B,C,D),Ts,'zoh');
[F,G,H,I,Ts] = ssdata(dsys);
R = sqrt(2)/2*[-1 1; 
              -1 -1];

%pole placement

poles = [0.96;0.961;0.9611;0.96111];
kpp = place(F, G, poles);


%% EKF initialization

P = [0.5 0 0 0;
     0 0.5 0 0;
     0 0 0.5 0;
     0 0 0 0.5];