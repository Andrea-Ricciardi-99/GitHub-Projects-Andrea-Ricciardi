clear all
close all
clc

%% COMPUTE TARGET AMPLITUDE

% import matrices:
load("matrices.mat");

% load the FRF data:
xB = load("FRFB.mat").x;
xB_reduced = load("FRFB_reduced.mat").x;
vect_freq = load("FRFB.mat").fre;


% compute the acceleration response:
for k=1:length(vect_freq)
    omega = 2*pi*vect_freq(k);

    xBdd = -omega^2*xB(idb(5,2),k); % compute acceleration
    modB(k) = abs(xBdd);

    xBdd_reduced = -omega^2*xB_reduced(idb(5,2),k);
    modB_reduced(k) = abs(xBdd_reduced);
end

figure
plot(vect_freq, modB); grid; title("FRF acc. B");

figure
plot(vect_freq, modB_reduced); grid; title("FRF acc. B reduced");

% get the maximum amplitude peak:
max_amp = max(modB)

% comute the target amplitude (-30%):
reduction = 0.3;
target_amp = max_amp*(1-reduction)

% maximum peak of the changed structure:
reduced_amp = max(modB_reduced)

