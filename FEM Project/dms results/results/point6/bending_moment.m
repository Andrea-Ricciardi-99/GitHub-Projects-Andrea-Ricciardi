clear all
close all
clc

%% LOAD DATA

% numerical data:
E = 2.06e11;
I = 2.313e-4;
EI = E*I;
rho = 7800;
A = 8.446e-3;
m = rho*A;
L = 5;

load("matrices.mat"); % import idb matrix

% import frequency responses of nodes A and B:
xA = load("FRFA.mat").x;
xB = load("FRFB.mat").x;
vect_freq = load("FRFA.mat").fre;

figure
subplot 311; plot(vect_freq, abs(xA(idb(4,2),:))); grid; title("wA");
subplot 312; plot(vect_freq, abs(xB(idb(5,2),:))); grid; title("wB");

% verify if wC is equal to the interpolated value:
wA = xA(idb(4,2),:);
wB = xB(idb(5,2),:);
thetaA = xA(idb(4,3),:);
thetaB = xB(idb(5,3),:);
wC = (wA + wB)/2 + (L/8)*(thetaA - thetaB); % response with interpolation

subplot 313; plot(vect_freq, abs(wC)); grid; title("wC");

%% FRF (BENDING MOMENT IN C)

% FRF responses of nodes A and B:
wA = xA(idb(4,2),:);
wB = xB(idb(5,2),:);
thetaA = xA(idb(4,3),:);
thetaB = xB(idb(5,3),:);

% second derivative of shape functions:
xsi = L/2;
f1 = 12*xsi/(L^3)-6/(L^2);
f2 = 6*xsi/(L^2)-4/L;
f3 = -12*xsi/(L^3)+6/(L^2);
f4 = 6*xsi/(L^2)-2/L;

% calculate displacement of C by interpolating with shape functions:
wCpp = f1*wA + f2*thetaA + f3*wB + f4*thetaB; % wCpp = -(1/L)*thetaA + (1/L)*thetaB;
Mc = EI*wCpp; % resultant bending moment

% plot the result:
figure
subplot 211; plot(vect_freq, abs(Mc)); grid; 
title("Bending moment at node C given force F at B"); xlabel("Freq. [Hz]"); ylabel("Amplitude [Nm/N]")
subplot 212; plot(vect_freq, angle(Mc)/pi*180); grid;
xlabel("Freq. [Hz]"); ylabel("Phase [°]")
