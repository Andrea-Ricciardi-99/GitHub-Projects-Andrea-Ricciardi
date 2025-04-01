clear all
close all
clc

%% LOAD DATA

load("project_mkr.mat");

modal_matrix = load("modes.mat").phi;
natural_freq = load("modes.mat").freq;

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

%% ADIMENTIONAL DAMPING RATIOS and DAMPED FREQUENCIES

Mmod = modal_matrix'*MFF*modal_matrix;
Kmod = modal_matrix'*KFF*modal_matrix;
Cmod = modal_matrix'*CFF*modal_matrix;

alpha = 0.8;
beta = 3.0e-5;

for i=1:length(natural_freq)
    omega = 2*pi*natural_freq(i);
    % adimensional damping ratios (from alpha and beta):
    h(i) = alpha/(2*omega) + (beta*omega)/2;
    % adimensional damping ratios (from modal damping matrix):
    %h(i) = Cmod(i,i) / (2*Mmod(i,i)*omega);
    % damped natural frequencies (wd=wn*sqrt(1-h^2)):
    damped_freq(i) = natural_freq(i)*sqrt(1-h(i)^2);
end

% undamped natural frequencies:
undamped_freq = natural_freq(natural_freq <= 24)'
% get the frequencies up to 24Hz:
damped_freq = damped_freq(damped_freq <= 24)
% relative adimensional damping ratios:
h = h(1:length(damped_freq))


