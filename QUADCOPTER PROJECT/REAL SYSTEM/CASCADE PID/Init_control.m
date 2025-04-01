%%%  AttitudeControl_tune
clear all
close all
clc
path('./icon/',path);
Init;

% constant value
RAD2DEG = 57.2957795;
DEG2RAD = 0.0174533;


%% Initial condition
ModelInit_PosE = [0, 0, -100];
ModelInit_VelB = [0, 0, 0];
ModelInit_AngEuler = [0, 0, 0];
ModelInit_RateB = [0, 0, 0];
ModelInit_Rads = 557.142;

% integral saturation
Saturation_I_RP_Max = 0.3;
Saturation_I_RP_Min = -0.3;
Saturation_I_Y_Max = 0.2;
Saturation_I_Y_Min = -0.2;

% max control angle, default 35deg
MAX_CONTROL_ANGLE_ROLL = 35;
MAX_CONTROL_ANGLE_PITCH  = 35;
% max control angle rate, rad/s 
MAX_CONTROL_ANGLE_RATE_PITCH = 220;
MAX_CONTROL_ANGLE_RATE_ROLL = 220;
MAX_CONTROL_ANGLE_RATE_Y = 200;

%% System Parameters (ESTIMATED BY US ON OUR ROBOT)
quadParams.g = 9.81;
quadParams.m = 1.07;
quadParams.Jx = 0.055;
quadParams.Jy = 0.055;
quadParams.Jz = 0.0366;
quadParams.Jr=0;
quadParams.d = 0.1;
quadParams.L = 0.225;
quadParams.kd= 1.779e-07;  %Rotor torque coefficient(kg.m^2)   %M=kd*w^2
quadParams.kt = 1.105e-05;  %Rotor thrust coefficient(kg.m^2)  %T=kt*w^2

% rotation matrix
R_p_x=sqrt(2)/2*[-1 1;
                 -1 -1];

% mapping u=B*PWM
B=[1 1 1 1;-1 1 1 -1; 1 -1 1 -1;1 1 -1 -1];

% mapping  PWM=B_inv*PWM
B_inv=inv(B);
%% 
s=tf('s');
Ts=0.002;

%% altitude controller design
G_z=1/s;
G_z_discrete=c2d(G_z,Ts,'zoh');
clear G_z
%% roll controller design
G_roll=1/s;
G_roll_discrete=c2d(G_roll,Ts,'tustin');
clear G_roll
%% pitch controller design
G_pitch=1/s;
G_pitch_discrete=c2d(G_pitch,Ts,'zoh');
clear G_pitch
%% yaw controller design
G_yaw=1/(quadParams.Jz*s^2);
G_yaw_discrete=c2d(G_yaw,Ts,'zoh');
clear G_yaw
%% rollRate controller design
G_rollRate=quadParams.L*sqrt(2)/2/(quadParams.Jx*s);
G_rollRate_discrete=c2d(G_rollRate,Ts,'zoh');
clear G_rollRate
%% pitchRate controller design
G_pitchRate=quadParams.L*sqrt(2)/2/(quadParams.Jy*s);
G_pitchRate_discrete=c2d(G_pitchRate,Ts,'zoh');
clear G_pitchRate
%% pitchRate controller design
G_yawRate=1/(quadParams.Jz*s);
G_yawRate_discrete=c2d(G_yawRate,Ts,'zoh');
clear G_yawRate


%% EKF initial condition

P = [0.5 0 0 0 0 0;
     0 0.5 0 0 0 0;
     0 0 0.5 0 0 0;
     0 0 0 0.5 0 0;
     0 0 0 0 0.5 0
     0 0 0 0 0 0.5];