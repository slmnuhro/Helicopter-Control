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


% Calculate reference for positive inputs
% input_ref = 0.5;
% idx = find(pos.input_mean == input_ref);
% angle_ref = pos.mean.pitch(idx);
% angVel_ref = pos.mean.angular_velocity(idx);

% POSITIVE SYSTEM
G = eye(3);
H = zeros(2,3);
sys_pos = ss(helicopterSysID_pos.A,[helicopterSysID_pos.B G],helicopterSysID_pos.C,[helicopterSysID_pos.D H]);
q1_p = 10e1; % increasing changes frequency
q2_p = 1e-4; % increasing increases amplitude
q3_p = 8e-6; % determines angular vel estimation ( lower than dot )
Q_pos = diag([q1_p,q2_p,q3_p]);
R_pos = 1e-3;

% Kalman estimation
[kalmf_pos,L_pos,P_pos] = kalman(sys_pos,Q_pos,R_pos,0);


% Calculate reference for negative inputs
input_ref = -0.5;
idx = find(neg.input_mean == input_ref);
angle_ref = neg.mean.pitch(idx);
angVel_ref = neg.mean.angular_velocity(idx);

sys_neg = ss(helicopterSysID_neg.A,[helicopterSysID_neg.B G],helicopterSysID_neg.C,[helicopterSysID_neg.D H]);
q1_n = 10e1;
q2_n = 5e-5;
q3_n = 8e-6;
Q_neg = diag([q1_n,q2_n,q3_n]);
R_neg = 1e-3;

% Kalman estimation
[kalmf_neg,L_neg,P_neg] = kalman(sys_neg,Q_neg,R_neg,0);

save("../stateEst_output.mat","L_pos","L_neg")

%% Initial guesses of coefficients
% Approx. length of pendulum
T_test = 1.9;       % [s]
length_m = 9.81*(T_test/(2*pi))^2; % [m]
fprintf('Approximate effective length of pendulum is %3.3f meters \n',length_m)

% Approx. Mass moment of inertia propeller
