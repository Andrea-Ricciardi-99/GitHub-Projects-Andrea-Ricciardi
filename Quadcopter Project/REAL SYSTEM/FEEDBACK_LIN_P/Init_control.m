%%%  AttitudeControl_Sim
clear all
close all
clc
path('./icon/',path);
Init;

%%System Parameters (ESTIMATED BY US ON OUR ROBOT)
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


%%System Parameters (OF THE SIMULATED ROBOT)
% quadParams.g = 9.81;
% quadParams.m = 1.4;
% quadParams.Jx = 0.0211;
% quadParams.Jy = 0.0219;
% quadParams.Jz = 0.0366;
% quadParams.Jr=0;
% quadParams.d = 0.0715;
% quadParams.L = 0.225;
% quadParams.kd= 1.779e-07;  %Rotor torque coefficient(kg.m^2)   %M=kd*w^2
% quadParams.kt = 1.105e-05;  %Rotor thrust coefficient(kg.m^2)  %T=kt*w^2



T_bar=quadParams.m*quadParams.g/2;
Ts = 0.1; % Sampling time
param_A=(quadParams.m*quadParams.g*quadParams.d-2*T_bar*quadParams.d)/quadParams.Jx;
param_B=quadParams.L/quadParams.Jx;
R = sqrt(2)/2*[-1 1 0; 
              -1 -1 0;
              0 0 1];

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
%% Initial condition
ModelInit_PosE = [0, 0, 0];
ModelInit_VelB = [0, 0, 0];
ModelInit_AngEuler = [0, 0, 0];
ModelInit_RateB = [0, 0, 0];
ModelInit_Rads = 0;
%% CONTROLLER PARAMETERS ON THE LINEARIZED SYSTEM
lincontroller.K2=-80;
lincontroller.K3=-80;
lincontroller.K4=-80;