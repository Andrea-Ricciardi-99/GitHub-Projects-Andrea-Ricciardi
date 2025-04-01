clear all
close all
clc

%% LOAD DATA

% load system matrices:
load("matrices.mat");

% load FRF of point A and B calculated on the previous point:
xA = load("FRFA.mat").x;
xB = load("FRFB.mat").x;
vect_freq = load("FRFA.mat").fre;

modal_matrix = load("modes.mat").phi; % modal matrix
natural_freq = load("modes.mat").freq; % natural frequencies

%% MATRIX PARTITIONING

n = length(idb)*3;
n_doc = 4*2; % 4 hinges -> 8 doc
n_dof = n - n_doc;

MFF = M(1:n_dof, 1:n_dof);
CFF = R(1:n_dof, 1:n_dof);
KFF = K(1:n_dof, 1:n_dof);

MFC = M(1:n_dof, n_dof+1:n);
CFC = R(1:n_dof, n_dof+1:n);
KFC = K(1:n_dof, n_dof+1:n);

MCF = M(n_dof+1:n, 1:n_dof);
CCF = R(n_dof+1:n, 1:n_dof);
KCF = K(n_dof+1:n, 1:n_dof);

MCC = M(n_dof+1:n, n_dof+1:n);
CCC = R(n_dof+1:n, n_dof+1:n);
KCC = K(n_dof+1:n, n_dof+1:n);

%%  MODAL APPROACH

% extract the first three modes:
modal_matrix = modal_matrix(:, 1:3);

% modal matrices:
Mmod = modal_matrix'*MFF*modal_matrix;
Kmod = modal_matrix'*KFF*modal_matrix;
Cmod = modal_matrix'*CFF*modal_matrix;

% damping modal matrix (alternative approach):
% alpha = 0.8;
% beta = 3.0e-5;
% 
% for k=1:3
%     Cmod(k,k) = alpha*Mmod(k,k) + beta*Kmod(k,k);
% end

F = zeros(length(natural_freq), 1);
F(idb(5, 2)) = 1; % force applied on node 5 in vertical direction: idb(5:2)
Fmod = modal_matrix'*F;

% modes superimposition:
i = sqrt(-1);

for k=1:length(vect_freq)
    % compute the modal coordinates:
    omega = 2*pi*vect_freq(k);
    A = (-omega^2*Mmod + i*omega*Cmod + Kmod);
    q = A \ Fmod;

    % superimposition response:
    x = modal_matrix*q;
    xdd = -omega^2*x;

    out1 = xdd(idb(4,2)); % vertical acceleration of point A (node4)
    out2 = xdd(idb(5,2)); % vertical acceleration of point B (node5)
    
    mod1(k) = abs(out1);
    fas1(k) = angle(out1);

    mod2(k) = abs(out2);
    fas2(k) = angle(out2);
end

%% PLOT THE RESULT

% import old results
for k=1:length(vect_freq)
    omega = 2*pi*vect_freq(k);

    xAdd = -omega^2*xA(idb(4,2),k);
    xBdd = -omega^2*xB(idb(5,2),k);

    modA(k) = abs(xAdd);
    fasA(k) = angle(xAdd);

    modB(k) = abs(xBdd);
    fasB(k) = angle(xBdd);
end

% acceleration A:
figure

subplot 211;
plot(vect_freq, modA, 'b');
hold on;
plot(vect_freq, mod1, 'r'); 
grid; xlabel('[Hz]'); ylabel('Amp.');
title('F/accA');
legend('system response', 'modal approach');

subplot 212; 
plot(vect_freq, fasA*180/pi, 'b');
hold on;
plot(vect_freq, fas1*180/pi, 'r');
grid; xlabel('[Hz]'); ylabel('Phase [deg]');
legend('system response', 'modal approach');

% acceleration B:
figure

subplot 211;
plot(vect_freq, modB, 'b'); 
hold on;
plot(vect_freq, mod2, 'r'); 
grid; xlabel('[Hz]'); ylabel('Amp.');
title('F/accB');
legend('system response', 'modal approach');

subplot 212; 
plot(vect_freq, fasB*180/pi, 'b');
hold on;
plot(vect_freq, fas2*180/pi, 'r');
grid; xlabel('[Hz]'); ylabel('Phase [deg]');
legend('system response', 'modal approach');
