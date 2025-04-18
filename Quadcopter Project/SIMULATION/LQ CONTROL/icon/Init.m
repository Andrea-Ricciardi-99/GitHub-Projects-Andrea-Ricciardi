%Initial condition
ModelInit_PosE = [0, 0, 0];  %Initial position
ModelInit_VelB = [0, 0, 0];  %Initial velocity
ModelInit_AngEuler = [0, 0, 0];  %Initial Euler angle
ModelInit_RateB = [0, 0, 0];  %Initial angular velocity
ModelInit_Rads = 0; %Initial motor speed(rad/s)

%UAV model parameter
ModelParam_uavMass = 1.07; %Mass of UAV(kg)
ModelParam_uavJxx = 0.055;
ModelParam_uavJyy = 0.055; 
ModelParam_uavJzz = 0.0366;
%Moment of inertia matrix
ModelParam_uavJ= [ModelParam_uavJxx, 0, 0;...
    0, ModelParam_uavJyy, 0;...
    0, 0, ModelParam_uavJzz];
ModelParam_uavType = int8(3); %X-type quadrotor��refer to "SupportedVehicleTypes.pdf" for specific definitions
ModelParam_uavMotNumbs = int8(4);  %Number of motors
ModelParam_uavR = 0.225;   %Body radius(m)
ModelParam_uavCd = 0.055;   %Damping coefficient(N/(m/s)^2)
ModelParam_uavCCm = [0.0035 0.0039 0.0034]; %Damping moment coefficient vector(N/(m/s)^2)

ModelParam_motorCr = 1148; %Motor throttle-speed curve slope(rad/s)
ModelParam_motorWb = -141.4;  %Motor speed-throttle curve constant term(rad/s)
ModelParam_motorT = 0.02;  %Motor inertia time constant(s)
ModelParam_motorJm = 0.0001287;  %Moment of inertia of motor rotor + propeller(kg.m^2)
ModelParam_motorMinThr = 0.05;  %Motor throttle dead zone(kg.m^2)
%M=Cm*w^2
ModelParam_rotorCm = 1.779e-07;  %Rotor torque coefficient(kg.m^2)
%T=Ct**w^2
ModelParam_rotorCt = 1.105e-05;  %Rotor thrust coefficient(kg.m^2)

%Environment Parameter
ModelParam_envGravityAcc = 9.8;  %Gravity acceleration(m/s^2)
ModelParam_envLongitude = 116.259368300000;  
ModelParam_envLatitude = 40.1540302;         
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];  %Latitude and longitude
ModelParam_envAltitude = -41.5260009765625;  %Reference height, down is positive
ModelParam_BusSampleRate = 0.001;  %Model sampling rate