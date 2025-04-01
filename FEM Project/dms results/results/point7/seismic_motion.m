clear all
close all
clc

%% LOAD DATA

% get constraint displacements:
data = load("seismic_displ.txt");
time = data(:, 1);
xc1 = data(:, 2); % displacement of points O11/O12
xc2 = data(:, 3); % displacement of points O21/O22

figure
subplot 211; plot(time, xc1); grid; xlabel("[s]"); ylabel("Y_{O_{11}/O_{12}} [m]"); title("Input seismic motion"); 
subplot 212; plot(time, xc2); grid; xlabel("[s]"); ylabel("Y_{O_{21}/O_{22}} [m]")

% get system matrices:
load("matrices.mat");

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

%% CONVERT TO FREQUENCY DOMAIN INPUT SIGNALS

% Sampling parameters:
dt = time(2)-time(1);   % Sampling distance
fs = 1/dt;              % Sampling frequency
N = length(time);       % Length of the signal (number of samples)
T = N/fs;               % Period of sampling
freq = (0:N-1)*(fs/N);  % Frequency vector (== (0:N-1)*(1/T))

% Perform FFT:
Xc1 = fft(xc1);
Xc2 = fft(xc2);

% Analysis of the spectrum of the signals (actually one-sided spectrums: values up to k=N/2-1):
Xc1_half = Xc1(1:N/2);
Xc2_half = Xc2(1:N/2);
freq_half = freq(1:N/2);

figure;
subplot 321; plot(time, xc1); grid;
title('Time-Domain Signal Y_{O11/O12}'); xlabel('Time [s]'); ylabel('Amplitude [m]');
subplot 323;
plot(freq_half, abs(Xc1_half)); grid;
title('Frequency-Domain Signal (Amplitude Spectrum)'); xlabel('Frequency [Hz]'); ylabel('Amplitude');
subplot 325;
plot(freq_half, angle(Xc1_half)); grid;
title('Frequency-Domain Signal (Phase Spectrum)'); xlabel('Frequency [Hz]'); ylabel('Phase');
subplot 322; plot(time, xc2); grid;
title('Time-Domain Signal Y_{O21/O22}'); xlabel('Time [s]'); ylabel('Amplitude  [m]');
subplot 324;
plot(freq_half, abs(Xc2_half)); grid;
title('Frequency-Domain Signal (Amplitude Spectrum)'); xlabel('Frequency [Hz]'); ylabel('Amplitude');
subplot 326;
plot(freq_half, angle(Xc2_half)); grid;
title('Frequency-Domain Signal (Phase Spectrum)'); xlabel('Frequency [Hz]'); ylabel('Phase');

%% FRF

i = sqrt(-1);

for k=1:(N/2)
    omega = 2*pi*freq(k); % omega = 2*pi*k/T

    A = -omega^2*MFF + i*omega*CFF + KFF;

    % constraint displacement vector (xc):
    x = zeros(n, 1); 
    x(idb(1,2),1) = Xc1(k); % coordinate of vertical displacement of O11 (node 1)
    x(idb(8,2),1) = Xc1(k); % coordinate of vertical displacement of O12 (node 8)
    x(idb(7,2),1) = Xc2(k); % coordinate of vertical displacement of O21 (node 7)
    x(idb(20,2),1) = Xc2(k); % coordinate of vertical displacement of O22 (node 20)
    xc = x(n_dof+1:end, 1);

    FFC = -(-omega^2*MFC + i*omega*CFC + KFC)*xc;

    % system response:
    X = A \ FFC;
    XA(k) = X(idb(4,2));
    XAdd(k) = -omega^2*XA(k);
end

% spectrum of the signal:
XA = [XA(1) XA(2:end) fliplr(conj(XA(2:end)))];
XAdd = [XAdd(1) XAdd(2:end) fliplr(conj(XAdd(2:end)))];
% time series:
xA = ifft(XA);
xAdd = ifft(XAdd);

% Confront response with inputs:
figure;
plot(time(1:end-1), xA); grid;
hold on;
plot(time, xc1); grid;
hold on;
plot(time, xc2); grid;
legend('y_A', 'O{11}/O{12}', 'O{21}/O{22}');
xlabel('Time [s]');ylabel('Displacement [m]');
title('Vertical displ. of A given the seism'); xlabel('Time [s]'); ylabel('Amplitude [m]');

% Plot the spectrum:
figure;
subplot 211; plot(freq(1:N/2+1), abs(XA(1:N/2+1))); grid;
title('Spectrum of the vertical displ. of A'); xlabel('Freq. [Hz]'); ylabel('Amplitude');
subplot 212; plot(freq(1:N/2+1), angle(XA(1:N/2+1))*180/pi); grid;
xlabel('Freq. [Hz]'); ylabel('Phase');

% Plot the time series:
figure;
subplot 211; plot(time(1:end-1), xA); grid;
title('Vertical displacement of A'); xlabel('Time. [s]'); ylabel('Amplitude [m]');
subplot 212; plot(time(1:end-1), xAdd); grid;
title('Vertical acceleration of A'); xlabel('Time. [s]'); ylabel('Amplitude [m/s^2]');

