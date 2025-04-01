clear all;
clc;
close all;

%% Load signals measurements

load("sensors.mat");

%% Extrapolate noise info

% Note: the real mean of the noise is of course 0, here we are just
% checking that the mean corresponds to the measure

% Accelerometer

% Mean
m_a_x = mean(sensors.accelerometer.signals.values(:,1));
m_a_y = mean(sensors.accelerometer.signals.values(:,2));
m_a_z = mean(sensors.accelerometer.signals.values(:,3));

% Variance
v_a_x = var(sensors.accelerometer.signals.values(:,1));
v_a_y = var(sensors.accelerometer.signals.values(:,2));
v_a_z = var(sensors.accelerometer.signals.values(:,3));

% Gyroscope

% Mean
m_g_x = mean(sensors.gyroscope.signals.values(:,1));
m_g_y = mean(sensors.gyroscope.signals.values(:,2));
m_g_z = mean(sensors.gyroscope.signals.values(:,3));

% Variance
v_g_x = var(sensors.gyroscope.signals.values(:,1));
v_g_y = var(sensors.gyroscope.signals.values(:,2));
v_g_z = var(sensors.gyroscope.signals.values(:,3));

% Quaternion

% Mean
m_q_x = mean(sensors.euler.signals.values(:,1));
m_q_y = mean(sensors.euler.signals.values(:,2));
m_q_z = mean(sensors.euler.signals.values(:,3));

% Variance
v_q_x = var(sensors.euler.signals.values(:,1));
v_q_y = var(sensors.euler.signals.values(:,2));
v_q_z = var(sensors.euler.signals.values(:,3));


%% Plots

figure;
plot(sensors.euler.time,sensors.euler.signals.values(:,1));
figure;
plot(sensors.euler.time,sensors.euler.signals.values(:,2));
figure;
plot(sensors.euler.time,sensors.euler.signals.values(:,3));

figure;
plot(sensors.accelerometer.time,sensors.accelerometer.signals.values(:,1));
figure;
plot(sensors.accelerometer.time,sensors.accelerometer.signals.values(:,2));
figure;
plot(sensors.accelerometer.time,sensors.accelerometer.signals.values(:,3));

figure;
plot(sensors.gyroscope.time,sensors.gyroscope.signals.values(:,1));
figure;
plot(sensors.gyroscope.time,sensors.gyroscope.signals.values(:,2));
figure;
plot(sensors.gyroscope.time,sensors.gyroscope.signals.values(:,3));

