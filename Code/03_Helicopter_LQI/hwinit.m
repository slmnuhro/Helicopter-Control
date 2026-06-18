%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DSCS FPGA interface board: init and I/O conversions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ======================================================================= %
% --------------------- SC42035 Integration Project --------------------- %
% -------------------------- Helicopter 2-A ----------------------------- %
% ----------------------------------------------------------------------- %
% ---------------------- Sven Rutgers 4600150 --------------------------- %
% ----------------------- Melis Orhun 4912071 --------------------------- %
% ======================================================================= %

close all
clear 
clc

% ======================================================================= %
% ----------------------------- PARAMETERS ------------------------------ %
% ======================================================================= %

% Sensor calibration
pitchoffs = 1.381e-2;
pitchgain = 1;
yawoffs = 1.685e-2;
yawgain = 1;

adinoffs = [pitchoffs 0 yawoffs 0];    % input offset
adingain = [pitchgain 1 yawgain 1];     % input gain (to radians)

% Define input parameters
par.elevation.amplitude = -0.1;      % [-1,1]
par.elevation.frequency = 1/200;    % [Hz]

par.azimuth.amplitude = 0;          % [-1, 1]
par.azimuth.frequency = 1/200;      % [Hz]

% Sampling time
h = 0.001;

load("../../Helicopter_01_SystemID/sysID_output.mat")
load("../../Helicopter_02_StateEstimation/stateEst_output.mat")

% Calculate reference for positive inputs
% input_ref = 0.5;
% idx = find(pos.input_mean == input_ref);
% angle_ref = pos.mean.pitch(idx);
% angVel_ref = pos.mean.angular_velocity(idx);

Q_LQR_pos = diag([10000,1000,0.5]);
R_LQR_pos = 1;
K_LQR_pos = lqr(helicopterSysID_pos.A,helicopterSysID_pos.B,Q_LQR_pos,R_LQR_pos,0);
Ki_pos = 5;

% Calculate reference for negative inputs
input_ref = -0.5;
idx = find(neg.input_mean == input_ref);
angle_ref = neg.mean.pitch(idx);
angVel_ref = neg.mean.angular_velocity(idx);


Q_LQR_neg = diag([10000,1000,1]);
R_LQR_neg = 1;
K_LQR_neg = lqr(helicopterSysID_neg.A,helicopterSysID_neg.B,Q_LQR_neg,R_LQR_neg,0);
Ki_neg = 1;
save("../LQR_output.mat","K_LQR_pos","K_LQR_neg")

%% Initial guesses of coefficients
% Approx. length of pendulum
T_test = 1.9;       % [s]
length_m = 9.81*(T_test/(2*pi))^2; % [m]
fprintf('Approximate effective length of pendulum is %3.3f meters \n',length_m)

% Approx. Mass moment of inertia propeller
