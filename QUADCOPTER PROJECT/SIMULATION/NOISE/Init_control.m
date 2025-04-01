%%%  AttitudeControl_tune
clear all
clc
path('./icon/',path);
Init;

% constant value
RAD2DEG = 57.2957795;
DEG2RAD = 0.0174533;
% throttle when UAV is hovering
THR_HOVER = 0.609;


%% Initial condition
ModelInit_PosE = [0, 0, -100];
ModelInit_VelB = [0, 0, 0];
ModelInit_AngEuler = [0.4, 0.4, 0];
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
d = 0.055;
J = 0.055;
g = 9.81;
Ts = 0.002;
T1 = 3;
T2 = 3;
T3 = 3;
T4 = 3;

A = [0 1;
    (m*g*d-T2*d-T1*d)/J 0];
B = [ 0; L/J];
C = sqrt(2)/2*[0 -1];
D = zeros(1,1);
dsys = c2d(ss(A,B,C,D),Ts,'zoh');
[F,G,H,I,Ts] = ssdata(dsys);
s = tf('s');
z = tf('z',Ts);
Gc = C*inv(s*eye(2)-A)*B;
Gd = c2d(Gc,Ts,'zoh');
inte = 1/(z-1);

%% EKF param

R = [0 0;
     0 0];
Q = zeros(4);
P = [0.5 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0.5 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0.5 0;
     0 0 0 0 0 0];

% P = [0.1 0 0 0;
%      0 0 0 0;
%      0 0 0.1 0;
%      0 0 0 0];

%% Simulated model
quadParams.g = 9.81;
quadParams.m = 1.07;
quadParams.Jx = 0.0156;
quadParams.Jy = 0.0156;
quadParams.Jz = 0.03;
quadParams.Jr=0;
quadParams.d = 0.1;
quadParams.L = 0.25;
quadParams.kd= 1.779e-07;  %Rotor torque coefficient(kg.m^2)   %M=kd*w^2
quadParams.kt = 1.105e-05;  %Rotor thrust coefficient(kg.m^2)  %T=kt*w^2

T_bar=quadParams.m*quadParams.g/2;
Ts = 0.02; % Sampling time
param_A=(quadParams.m*quadParams.g*quadParams.d-2*T_bar*quadParams.d)/quadParams.Jx;
param_B=quadParams.L/quadParams.Jx;

R_p_x=sqrt(2)/2*[-1 1;
                 -1 -1];
%Constant value
RAD2DEG = 57.2957795;
DEG2RAD = 0.0174533;
%throttle when UAV is hovering
THR_HOVER = 0.609;

%% Motor Mixer matrix T=quadParams.MotorMixer*U
const1=quadParams.L*sqrt(2)/2;
const2=quadParams.kd/quadParams.kt;
T_to_U=[  1       1      1       1;
        -const1  const1  const1  -const1;
        const1  -const1  const1  -const1;
         const2  const2  -const2   -const2];
quadParams.MotorMixer=inv(T_to_U);

%% Poles

lincontroller.K2=-60;
lincontroller.K3=-60;
lincontroller.K4=-60;

%% run simulink model
%POLEPLACEMENT_190424
%AttitudeControl_Sim