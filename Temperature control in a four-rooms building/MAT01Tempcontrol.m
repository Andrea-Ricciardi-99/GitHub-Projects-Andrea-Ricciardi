%% Temperature control example

clear
clc
close all

%% Variables definitions

Ts = 400; %temperature in a room changes slowly
N = 4;

alpha = 0.004;

qc = 0.005;
rc = 1000;

qd = 0.01;
rd = 1;

%sector variables 

theta_s = pi/4;

%state space matrices definition

gamma_1=6.22e-4;
gamma_2=2.5e-4;
gamma_e=2.5e-4;
gamma_ol=gamma_1+gamma_2+gamma_e;

A=[-gamma_ol gamma_2 gamma_1 0
    gamma_2 -gamma_ol 0 gamma_1
    gamma_1 0 -gamma_ol gamma_2
    0 gamma_1 gamma_2 -gamma_ol];
B=eye(4);
C=eye(4);
D=zeros(4);

%% Splitting the continuous time matrices

%write the decomposed system on the presentation defining B1,B2 and so on

B1 = B(:,1);
B2 = B(:,2);
B3 = B(:,3);
B4 = B(:,4);

C1 = C(1,:);
C2 = C(2,:);
C3 = C(3,:);
C4 = C(4,:);

B_comp = {B1 B2 B3 B4};
C_comp = {C1;C2;C3;C4};

%% Obtaining the system state space representations

sysCT = ss(A,B,C,D);

sysDT = c2d(sysCT,Ts,'zoh');

[F,G,H,L,Ts] = ssdata(sysDT);

%% Splitting the discrete time matrices

G1 = G(:,1);
G2 = G(:,2);
G3 = G(:,3);
G4 = G(:,4);

H1 = H(1,:);
H2 = H(2,:);
H3 = H(3,:);
H4 = H(4,:);

G_comp = {G1 G2 G3 G4};
H_comp = {H1;H2;H3;H4};

%% Continous time system analysis

Ec = eig(A);                %from the eigenvalues we can detect it's AS because of
rhoc = max(real(eig(A)));   %the negative real value of the eigen value even though
                            %one of them is near the origin

%% Discrete time system analysis

Ed = eig(F);                %from the the eigenvalues we can detect it's simple
rhod = max(abs(eig(F)));    %stable since one of the eigenvalues is on the UC

%% Plotting the eigenvalues

%continuous time

% Plot the eigenvalues in the complex plane
figure(17);
subplot(2,1,1);
plot(real(Ec), imag(Ec), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

%discrete time

subplot(2,1,2);
plot(real(Ed), imag(Ed), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

sgtitle('Plant');

%% Centralized case
%let's start by doing the centralized case

ContStruc_c = ones(N,N);

Cfm_c = di_fixed_modes( A, B_comp, C_comp, N, ContStruc_c, 2);
Cfm_d = di_fixed_modes( F, G_comp, H_comp, N, ContStruc_c, 2);

[K_cc,rho_cc,feas_cc] = LMI_CT_DeDicont(A,B_comp,C_comp,N,ContStruc_c);
[K_dc,rho_dc,feas_dc] = LMI_DT_DeDicont(F,G_comp,H_comp,N,ContStruc_c);

[K_cc_a, rho_cc_a, feas_cc_a] = LMI_CT_DeDicont_perf_alpha(A,B_comp,C_comp,N,ContStruc_c,alpha);
[K_dc_p, rho_dc_p, feas_dc_p] = LMI_DT_DeDicont_perf_p(F,G_comp,H_comp,N,ContStruc_c,0.5);

[K_cc_s, rho_cc_s, feas_cc_s] = LMI_CT_DeDicont_sector(A,B_comp,C_comp,N,ContStruc_c,theta_s);

[K_cc_h2, rho_cc_h2, feas_cc_h2] = LMI_CT_DeDicont_H2(A,B_comp,C_comp,N,ContStruc_c,qc,rc);
[K_dc_h2, rho_dc_h2, feas_dc_h2] = LMI_DT_DeDicont_H2(F,G_comp,H_comp,N,ContStruc_c,qd,rd);


%% Trajectories for centralized case

x0 = [2; 1; 0.5; 0.7;]; %Initial states

%discrete time 
FCL_1 = (F + K_dc*G);           %closed loop stability LMI
FCL_2 = (F + K_dc_p*G);         %closed loop performance LMI
FCL_3 = (F + K_dc_h2*G);        %closed loop H2 minimization

x_lmi_1 = zeros(4,20);
x_lmi_2 = zeros(4,20);
x_lmi_3 = zeros(4,20);

x_lmi_1(:,1) = x0;
x_lmi_2(:,1) = x0;
x_lmi_3(:,1) = x0;

for i = 1:20
    x_lmi_1(:,i+1) = FCL_1*x_lmi_1(:,i);
    x_lmi_2(:,i+1) = FCL_2*x_lmi_2(:,i);
    x_lmi_3(:,i+1) = FCL_3*x_lmi_3(:,i);
end

figure(1);
subplot(3,1,1);
plot((0:20)*Ts, x_lmi_1);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with Standard LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(3,1,2);
plot((0:20)*Ts, x_lmi_2);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with Performance LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(3,1,3);
plot((0:20)*Ts, x_lmi_3);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with H2 minimization');
legend('y1', 'y2','y3','y4');  
grid on;

sgtitle('Centralized case');

%continous time

ACL_1 = (A+K_cc*B);             %closed loop stability LMI
ACL_2 = (A+K_cc_a*B);           %closed loop performance LMI
ACL_3 = (A+K_cc_s*B);           %closed loop sector LMI
ACL_4 = (A+K_cc_h2*B);          %closed loop H2 minimization
BCL = zeros(4);
CCL = C;
DCL = D;

sysCT_CL_1 = ss(ACL_1, BCL, CCL, DCL);
sysCT_CL_2 = ss(ACL_2, BCL, CCL, DCL);
sysCT_CL_3 = ss(ACL_3, BCL, CCL, DCL);
sysCT_CL_4 = ss(ACL_4, BCL, CCL, DCL);

t = linspace(0, 10, 1000); % Adjust the time span and resolution as needed

[y1, t1] = lsim(sysCT_CL_1, zeros(length(t),4), t, x0);

t = linspace(0, 2000, 200000);

[y2, t2] = lsim(sysCT_CL_2, zeros(length(t),4), t, x0);
[y3, t3] = lsim(sysCT_CL_3, zeros(length(t),4), t, x0);
[y4, t4] = lsim(sysCT_CL_4, zeros(length(t),4), t, x0);

figure(2);
subplot(4,1,1);
plot(t1, y1);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Standard LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,2);
plot(t2, y2);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Performance LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,3);
plot(t3, y3);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Sector LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,4);
plot(t4, y4);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with H2 minimization');
legend('y1', 'y2','y3','y4');  
grid on;

sgtitle('Centralized case');

%% Eigenvalues for the closed loop [centralized case]

%continous time case
eig_v_cc = eig(ACL_1);          %stability LMI case
eig_v_cc_a = eig(ACL_2);        %performance LMI case
eig_v_cc_s = eig(ACL_3);        %sector LMI case
eig_v_cc_h2 = eig(ACL_4);       %H2 minimization case

%discrete time case
eig_v_dc = eig(FCL_1);          %stability LMI case
eig_v_dc_p = eig(FCL_2);        %performance LMI case
eig_v_dc_h2 = eig(FCL_3);       %H2 minimization case  

%% Plotting the eigenvalues

%continuous time

% Plot the eigenvalues in the complex plane
figure(3);
subplot(4,1,1);
plot(real(eig_v_cc), imag(eig_v_cc), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Standard LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,2);
plot(real(eig_v_cc_a), imag(eig_v_cc_a), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Performance LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,3);
plot(real(eig_v_cc_s), imag(eig_v_cc_s), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Sector LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,4);
plot(real(eig_v_cc_h2), imag(eig_v_cc_h2), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with H2 minimization');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

sgtitle('Centralized case');

%discrete time

% Plot the eigenvalues in the complex plane
figure(4);
subplot(3,1,1);
plot(real(eig_v_dc), imag(eig_v_dc), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with Standard LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

subplot(3,1,2);
plot(real(eig_v_dc_p), imag(eig_v_dc_p), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with Performance LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

subplot(3,1,3);
plot(real(eig_v_dc_h2), imag(eig_v_dc_h2), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with H2 Minimization');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

sgtitle('Centralized case');

%% Decentralized case

ConstStruc_de = eye(N);

Defm_c = di_fixed_modes( A, B_comp, C_comp, N, ConstStruc_de, 2);  
Defm_d = di_fixed_modes( F, G_comp, H_comp, N, ConstStruc_de, 2);

[K_cde,rho_cde,feas_cde] = LMI_CT_DeDicont(A,B_comp,C_comp,N,ConstStruc_de);
[K_dde,rho_dde,feas_dde] = LMI_DT_DeDicont(F,G_comp,H_comp,N,ConstStruc_de);

[K_cde_a, rho_cde_a, feas_cde_a] = LMI_CT_DeDicont_perf_alpha(A,B_comp,C_comp,N,ConstStruc_de,alpha);
[K_dde_p, rho_dde_p, feas_dde_p] = LMI_DT_DeDicont_perf_p(F,G_comp,H_comp,N,ConstStruc_de,0.5);

[K_cde_s, rho_cde_s, feas_cde_s] = LMI_CT_DeDicont_sector(A,B_comp,C_comp,N,ConstStruc_de,theta_s);

[K_cde_h2, rho_cde_h2, feas_cde_h2] = LMI_CT_DeDicont_H2(A,B_comp,C_comp,N,ConstStruc_de, qc, rc);
[K_dde_h2, rho_dde_h2, feas_dde_h2] = LMI_DT_DeDicont_H2(F,G_comp,H_comp,N,ConstStruc_de, qd, rd);


%% Trajectories for decentralized

x0 = [2; 1; 0.5; 0.7;]; %Initial states

%discrete time 
FCL_1 = (F + K_dde*G);          %closed loop stability LMI
FCL_2 = (F + K_dde_p*G);        %closed loop performance LMI
FCL_3 = (F + K_dde_h2*G);       %closed loop H2 minimization

x_lmi_1 = zeros(4,20);
x_lmi_2 = zeros(4,20);
x_lmi_3 = zeros(4,20);

x_lmi_1(:,1) = x0;
x_lmi_2(:,1) = x0;
x_lmi_3(:,1) = x0;

for i = 1:20
    x_lmi_1(:,i+1) = FCL_1*x_lmi_1(:,i);
    x_lmi_2(:,i+1) = FCL_2*x_lmi_2(:,i);
    x_lmi_3(:,i+1) = FCL_3*x_lmi_3(:,i);
end

figure(5);
subplot(3,1,1);
plot((0:20)*Ts, x_lmi_1);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with Standard LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(3,1,2);
plot((0:20)*Ts, x_lmi_2);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with Performance LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(3,1,3);
plot((0:20)*Ts, x_lmi_3);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with H2 minimization LMI');
legend('y1', 'y2','y3','y4');  
grid on;

sgtitle('Decentralized case');

%continous time

ACL_1 = (A+K_cde*B);
ACL_2 = (A+K_cde_a*B);
ACL_3 = (A+K_cde_s*B);
ACL_4 = (A+K_cde_h2*B);
BCL = zeros(4);
CCL = C;
DCL = D;

sysCT_CL_1 = ss(ACL_1, BCL, CCL, DCL);
sysCT_CL_2 = ss(ACL_2, BCL, CCL, DCL);
sysCT_CL_3 = ss(ACL_3, BCL, CCL, DCL);
sysCT_CL_4 = ss(ACL_4, BCL, CCL, DCL);

t = linspace(0, 10, 1000); % Adjust the time span and resolution as needed

[y1, t1] = lsim(sysCT_CL_1, zeros(length(t),4), t, x0);

t = linspace(0, 2000, 200000);

[y2, t2] = lsim(sysCT_CL_2, zeros(length(t),4), t, x0);
[y3, t3] = lsim(sysCT_CL_3, zeros(length(t),4), t, x0);
[y4, t4] = lsim(sysCT_CL_4, zeros(length(t),4), t, x0);

figure(6);
subplot(4,1,1);
plot(t1, y1);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Standard LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,2);
plot(t2, y2);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Performance LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,3);
plot(t3, y3);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Sector LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,4);
plot(t4, y4);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with H2 minimization');
legend('y1', 'y2','y3','y4');  
grid on;

sgtitle('Decentralized case');

%% Eigenvalues for the closed loop [decentralized case]

%continous time case
eig_v_cde = eig(ACL_1);          %stability LMI case
eig_v_cde_a = eig(ACL_2);        %performance LMI case
eig_v_cde_s = eig(ACL_3);        %sector LMI case
eig_v_cde_h2 = eig(ACL_4);       %H2 minimization case

%discrete time case
eig_v_dde = eig(FCL_1);          %stability LMI case
eig_v_dde_p = eig(FCL_2);        %performance LMI case
eig_v_dde_h2 = eig(FCL_3);       %H2 minimization case 

%% Plotting the eigenvalues

%continuous time

% Plot the eigenvalues in the complex plane
figure(7);
subplot(4,1,1);
plot(real(eig_v_cde), imag(eig_v_cde), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Standard LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,2);
plot(real(eig_v_cde_a), imag(eig_v_cde_a), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Performance LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,3);
plot(real(eig_v_cde_s), imag(eig_v_cde_s), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Sector LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,4);
plot(real(eig_v_cde_h2), imag(eig_v_cde_h2), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with H2 minimization');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

sgtitle('Decentralized case');

%discrete time

% Plot the eigenvalues in the complex plane
figure(8);
subplot(3,1,1);
plot(real(eig_v_dde), imag(eig_v_dde), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with Standard LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

subplot(3,1,2);
plot(real(eig_v_dde_p), imag(eig_v_dde_p), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with Performance LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

subplot(3,1,3);
plot(real(eig_v_dde_h2), imag(eig_v_dde_h2), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with H2 Minimization');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

sgtitle('Decentralized case');

%% Distributed case (case 1)
%let's consider the first case in which each room communicates with its
%neighbours

ConstStruc_di_1 = [1 1 1 0;
                 1 1 0 1;
                 1 0 1 1;
                 0 1 1 1;];

Di_1_fm_c = di_fixed_modes( A, B_comp, C_comp, N, ConstStruc_di_1, 2);  
Di_1_fm_d = di_fixed_modes( F, G_comp, H_comp, N, ConstStruc_di_1, 2);

[K_cdi_1,rho_cdi_1,feas_cdi_1] = LMI_CT_DeDicont(A,B_comp,C_comp,N,ConstStruc_di_1);
[K_ddi_1,rho_ddi_1,feas_ddi_1] = LMI_DT_DeDicont(F,G_comp,H_comp,N,ConstStruc_di_1);

[K_cdi_a_1, rho_cdi_a_1, feas_cdi_a_1] = LMI_CT_DeDicont_perf_alpha(A,B_comp,C_comp,N,ConstStruc_di_1,alpha);
[K_ddi_p_1, rho_ddi_p_1, feas_ddi_p_1] = LMI_DT_DeDicont_perf_p(F,G_comp,H_comp,N,ConstStruc_di_1,0.5);

[K_cdi_s_1, rho_cdi_s_1, feas_cdi_s_1] = LMI_CT_DeDicont_sector(A,B_comp,C_comp,N,ConstStruc_di_1,theta_s);

[K_cdi_h2_1, rho_cdi_h2_1, feas_cdi_h2_1] = LMI_CT_DeDicont_H2(A,B_comp,C_comp,N,ConstStruc_di_1,qc,rc);
[K_ddi_h2_1, rho_ddi_h2_1, feas_ddi_h2_1] = LMI_DT_DeDicont_H2(F,G_comp,H_comp,N,ConstStruc_di_1,qd,rd);

%% Trajectories for distributed (1st case)

x0 = [2; 1; 0.5; 0.7;]; %Initial states

%discrete time 
FCL_1 = (F + K_ddi_1*G);            %closed loop stability LMI
FCL_2 = (F + K_ddi_p_1*G);          %closed loop performance LMI
FCL_3 = (F + K_ddi_h2_1*G);         %closed loop H2 minimization

x_lmi_1 = zeros(4,20);
x_lmi_2 = zeros(4,20);
x_lmi_3 = zeros(4,20);

x_lmi_1(:,1) = x0;
x_lmi_2(:,1) = x0;
x_lmi_3(:,1) = x0;

for i = 1:20
    x_lmi_1(:,i+1) = FCL_1*x_lmi_1(:,i);
    x_lmi_2(:,i+1) = FCL_2*x_lmi_2(:,i);
    x_lmi_3(:,i+1) = FCL_3*x_lmi_3(:,i);
end

figure(9);
subplot(3,1,1);
plot((0:20)*Ts, x_lmi_1);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with Standard LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(3,1,2);
plot((0:20)*Ts, x_lmi_2);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with Performance LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(3,1,3);
plot((0:20)*Ts, x_lmi_3);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with H2 minimization');
legend('y1', 'y2','y3','y4');  
grid on;

sgtitle('Distributed case (case 1)');

%continous time

ACL_1 = (A+K_cdi_1*B);
ACL_2 = (A+K_cdi_a_1*B);
ACL_3 = (A+K_cdi_s_1*B);
ACL_4 = (A+K_cdi_h2_1*B);

BCL = zeros(4);
CCL = C;
DCL = D;

sysCT_CL_1 = ss(ACL_1, BCL, CCL, DCL);
sysCT_CL_2 = ss(ACL_2, BCL, CCL, DCL);
sysCT_CL_3 = ss(ACL_3, BCL, CCL, DCL);
sysCT_CL_4 = ss(ACL_4, BCL, CCL, DCL);

t = linspace(0, 10, 1000); % Adjust the time span and resolution as needed

[y1, t1] = lsim(sysCT_CL_1, zeros(length(t),4), t, x0);

t = linspace(0, 2000, 200000);

[y2, t2] = lsim(sysCT_CL_2, zeros(length(t),4), t, x0);
[y3, t3] = lsim(sysCT_CL_3, zeros(length(t),4), t, x0);
[y4, t4] = lsim(sysCT_CL_4, zeros(length(t),4), t, x0);

figure(10);
subplot(4,1,1);
plot(t1, y1);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Standard LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,2);
plot(t2, y2);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Performance LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,3);
plot(t3, y3);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Sector LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,4);
plot(t4, y4);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with H2 minimization');
legend('y1', 'y2','y3','y4');  
grid on;

sgtitle('Distributed case (case 1)');

%% Eigenvalues for the closed loop [distributed case 1]

%continous time case
eig_v_cdi_1 = eig(ACL_1);          %stability LMI case
eig_v_cdi_a_1 = eig(ACL_2);        %performance LMI case
eig_v_cdi_s_1 = eig(ACL_3);        %sector LMI case
eig_v_cdi_h2_1 = eig(ACL_4);       %H2 minimization case

%discrete time case
eig_v_ddi_1 = eig(FCL_1);          %stability LMI case
eig_v_ddi_p_1 = eig(FCL_2);        %performance LMI case
eig_v_ddi_h2_1 = eig(FCL_3);       %H2 minimization case 

%% Plotting the eigenvalues

%continuous time

% Plot the eigenvalues in the complex plane
figure(11);
subplot(4,1,1);
plot(real(eig_v_cdi_1), imag(eig_v_cdi_1), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Standard LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,2);
plot(real(eig_v_cdi_a_1), imag(eig_v_cdi_a_1), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Performance LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,3);
plot(real(eig_v_cdi_s_1), imag(eig_v_cdi_s_1), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Sector LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,4);
plot(real(eig_v_cdi_h2_1), imag(eig_v_cdi_h2_1), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with H2 minimization');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

sgtitle('Distributed case (case 1)');

%discrete time

% Plot the eigenvalues in the complex plane
figure(12);
subplot(3,1,1);
plot(real(eig_v_ddi_1), imag(eig_v_ddi_1), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with Standard LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

subplot(3,1,2);
plot(real(eig_v_ddi_p_1), imag(eig_v_ddi_p_1), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with Performance LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

subplot(3,1,3);
plot(real(eig_v_ddi_h2_1), imag(eig_v_ddi_h2_1), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with H2 Minimization');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

sgtitle('Distributed case (case 1)');

%% Distributed case (case 2)
%let's consider the case in which each room communicates just with the
%opposite with respect to itself


ConstStruc_di_2 = [1 0 0 1;
                 0 1 1 0;
                 0 1 1 0;
                 1 0 0 1;];
Di_2_fm_c = di_fixed_modes( A, B_comp, C_comp, N, ConstStruc_di_2, 2);  
Di_2_fm_d = di_fixed_modes( F, G_comp, H_comp, N, ConstStruc_di_2, 2);

[K_cdi_2,rho_cdi_2,feas_cdi_2] = LMI_CT_DeDicont(A,B_comp,C_comp,N,ConstStruc_di_2);
[K_ddi_2,rho_ddi_2,feas_ddi_2] = LMI_DT_DeDicont(F,G_comp,H_comp,N,ConstStruc_di_2);

[K_cdi_a_2, rho_cdi_a_2, feas_cdi_a_2] = LMI_CT_DeDicont_perf_alpha(A,B_comp,C_comp,N,ConstStruc_di_2,alpha);
[K_ddi_p_2, rho_ddi_p_2, feas_ddi_p_2] = LMI_DT_DeDicont_perf_p(F,G_comp,H_comp,N,ConstStruc_di_2,0.5);

[K_cdi_s_2, rho_cdi_s_2, feas_cdi_s_2] = LMI_CT_DeDicont_sector(A,B_comp,C_comp,N,ConstStruc_di_2,theta_s);

[K_cdi_h2_2, rho_cdi_h2_2, feas_cdi_h2_2] = LMI_CT_DeDicont_H2(A,B_comp,C_comp,N,ConstStruc_di_2,qc,rc);
[K_ddi_h2_2, rho_ddi_h2_2, feas_ddi_h2_2] = LMI_DT_DeDicont_H2(F,G_comp,H_comp,N,ConstStruc_di_2,qd,rd);

%% Trajectories for distributed (2nd case)

x0 = [2; 1; 0.5; 0.7;]; %Initial states

%discrete time 
FCL_1 = (F + K_ddi_2*G);            %closed loop stability LMI
FCL_2 = (F + K_ddi_p_2*G);          %closed loop performance LMI
FCL_3 = (F + K_ddi_h2_2*G);         %closed loop H2 minimization

x_lmi_1 = zeros(4,20);
x_lmi_2 = zeros(4,20);
x_lmi_3 = zeros(4,20);

x_lmi_1(:,1) = x0;
x_lmi_2(:,1) = x0;
x_lmi_3(:,1) = x0;

for i = 1:20
    x_lmi_1(:,i+1) = FCL_1*x_lmi_1(:,i);
    x_lmi_2(:,i+1) = FCL_2*x_lmi_2(:,i);
    x_lmi_3(:,i+1) = FCL_2*x_lmi_3(:,i);
end

figure(13);
subplot(3,1,1);
plot((0:20)*Ts, x_lmi_1);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with Standard LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(3,1,2);
plot((0:20)*Ts, x_lmi_2);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with Performance LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(3,1,3);
plot((0:20)*Ts, x_lmi_3);
xlabel('Time');
ylabel('Outputs');
title('Discrete Time Trajectories with H2 minimization');
legend('y1', 'y2','y3','y4');  
grid on;

sgtitle('Distributed case (case 2)');

%continous time

ACL_1 = (A+K_cdi_2*B);
ACL_2 = (A+K_cdi_a_2*B);
ACL_3 = (A+K_cdi_s_2*B);
ACL_4 = (A+K_cdi_h2_2*B);
BCL = zeros(4);
CCL = C;
DCL = D;

sysCT_CL_1 = ss(ACL_1, BCL, CCL, DCL);
sysCT_CL_2 = ss(ACL_2, BCL, CCL, DCL);
sysCT_CL_3 = ss(ACL_3, BCL, CCL, DCL);
sysCT_CL_4 = ss(ACL_4, BCL, CCL, DCL);

t = linspace(0, 10, 1000); % Adjust the time span and resolution as needed

[y1, t1] = lsim(sysCT_CL_1, zeros(length(t),4), t, x0);

t = linspace(0, 2000, 200000);

[y2, t2] = lsim(sysCT_CL_2, zeros(length(t),4), t, x0);
[y3, t3] = lsim(sysCT_CL_3, zeros(length(t),4), t, x0);
[y4, t4] = lsim(sysCT_CL_4, zeros(length(t),4), t, x0);

figure(14);
subplot(4,1,1);
plot(t1, y1);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Standard LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,2);
plot(t2, y2);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with Performance LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,3);
plot(t3, y3);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with sector LMI');
legend('y1', 'y2','y3','y4');  
grid on;

subplot(4,1,4);
plot(t4, y4);
xlabel('Time');
ylabel('Outputs');
title('Continuous Time Trajectories with H2 minimization');
legend('y1', 'y2','y3','y4');  
grid on;

sgtitle('Distributed case (case 2)');

%% Eigenvalues for the closed loop [distributed case 2]

%%continous time case
eig_v_cdi_2 = eig(ACL_1);          %stability LMI case
eig_v_cdi_a_2 = eig(ACL_2);        %performance LMI case
eig_v_cdi_s_2 = eig(ACL_3);        %sector LMI case
eig_v_cdi_h2_2 = eig(ACL_4);       %H2 minimization case

%%discrete time case
eig_v_ddi_2 = eig(FCL_1);          %stability LMI case
eig_v_ddi_p_2 = eig(FCL_2);        %performance LMI case
eig_v_ddi_h2_2 = eig(FCL_3);       %H2 minimization case 

%% Plotting the eigenvalues

%continuous time

% Plot the eigenvalues in the complex plane
figure(15);
subplot(4,1,1);
plot(real(eig_v_cdi_2), imag(eig_v_cdi_2), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Standard LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,2);
plot(real(eig_v_cdi_a_2), imag(eig_v_cdi_a_2), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Performance LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,3);
plot(real(eig_v_cdi_s_2), imag(eig_v_cdi_s_2), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with Sector LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

subplot(4,1,4);
plot(real(eig_v_cdi_h2_2), imag(eig_v_cdi_h2_2), 'o');
hold on;

% Plot a vertical line at the instability limit (real part becomes positive)
instability_limit = 0; % Set the instability limit
plot([instability_limit, instability_limit], ylim, 'r--', 'LineWidth', 2);

title('Continuous Time Eigenvalues with H2 minimization');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Instability Limit', 'Location', 'Best');
hold off;

sgtitle('Distributed case (case 2)');

%discrete time

% Plot the eigenvalues in the complex plane
figure(16);
subplot(3,1,1);
plot(real(eig_v_ddi_2), imag(eig_v_ddi_2), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with Standard LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

subplot(3,1,2);
plot(real(eig_v_ddi_p_2), imag(eig_v_ddi_p_2), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with Performance LMI');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

subplot(3,1,3);
plot(real(eig_v_ddi_h2_2), imag(eig_v_ddi_h2_2), 'o');
hold on;

% Plot the unit circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);

title('Discrete Time Eigenvalues with H2 Minimization');
xlabel('Real Part');
ylabel('Imaginary Part');
grid on;
legend('Eigenvalues', 'Unit Circle', 'Location', 'Best');
hold off;

sgtitle('Distributed case (case 2)');
