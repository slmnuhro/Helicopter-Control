% ======================================================================= %
% --------------------- SC42035 Integration Project --------------------- %
% -------------------------- Helicopter 2-A ----------------------------- %
% ----------------------------------------------------------------------- %
% ---------------------- Sven Rutgers 4600150 --------------------------- %
% ----------------------- Melis Orhun 4912071 --------------------------- %
% ======================================================================= %


% ======================================================================= %
% ----------------------- System Identification ------------------------- %
% ======================================================================= %

clc
clear
close all

% ======================================================================= %
% --------------------------- INITIALIZATION ---------------------------- %
% ======================================================================= %

% Set interpreter and layout options
set(groot,'defaulttextinterpreter','latex');  
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaultLegendInterpreter','latex'); 
set(groot,'defaultLineLineWidth',0.8)
set(groot,'defaultAxesFontSize',11)
set(groot,'defaultAxesFontWeight',"normal")

% Create Directory for plots
if isunix
    command = 'rm -r helicopter_sysID_plots';
elseif ispc
    command = 'rmdir /s /q helicopter_sysID_plots';
elseif ismac
    command = 'rm -r helicopter_sysID_plots';
end

status = system(command);
if (status ~= 0)
    disp("Failed to remove 'helicopter_sysID_plots' directory. Please remove it manually")
end

command = 'mkdir helicopter_sysID_plots';
status = system(command);
if (status ~= 0)
    disp("Failed to create 'helicopter_sysID_plots' directory. Please create it manually")
end

% ======================================================================= %
% ------------------------ PARAMETER DEFINITIONS ------------------------ %
% ======================================================================= %

% Define low pass filter parameters
sysID_par.Ts = 0.001; % [s] Sampling time
sysID_par.fpass = 0.1; % [Hz] Passband frequency
sysID_par.fs = 1/sysID_par.Ts; % [Hz] Sampling rate


load("id_data/chirp_02.mat")
% load("id_data/step_pos_id_5.mat")
    
% Read variables
time = sensor_out.Time; % [s] Run time
input = ScopeData1.signals(1).values; % Normalized input
pitch_ = movmean(sensor_out.Data(:,1),3); % [radians] Pitch angle
angular_velocity_ = sensor_out.Data(:,2); % [rpm] Angular velocity


% Apply FFT on the signals
sysID_fft.X_p_ = fftshift(fft(pitch_));
sysID_fft.X_v_ = fftshift(fft(angular_velocity_));
sysID_fft.N = length(pitch_);

sysID_fft.dF = sysID_par.fs/sysID_fft.N;
sysID_fft.f = -sysID_par.fs/2 : sysID_fft.dF : sysID_par.fs/2 - sysID_fft.dF;

% Apply low pass filter on the variables
pitch = lowpass(pitch_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);
angular_velocity = lowpass(angular_velocity_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);

sysID_fft.X_p = fftshift(fft(pitch));
sysID_fft.X_v = fftshift(fft(angular_velocity));

% Plots 
f = figure('Name',"SysID FFT");
subplot(2,1,1)
hold on
plot(sysID_fft.f,20.*log10(abs(sysID_fft.X_p_)));
plot(sysID_fft.f,20.*log10(abs(sysID_fft.X_p)));
grid minor
title("FFT of Pitch Angle")
legend("Unfiltered","Filtered")
xlim([-100,100])
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
subplot(2,1,2)
hold on
plot(sysID_fft.f,20.*log10(abs(sysID_fft.X_v_)));
plot(sysID_fft.f,20.*log10(abs(sysID_fft.X_v)));
grid minor
title("FFT of Angular Velocity")
legend("Unfiltered","Filtered")
xlim([-100,100])
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
saveas(f, 'helicopter_sysID_plots/fft','epsc');

% Apply low pass filter on the variables
pitch = lowpass(pitch_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);
angular_velocity = lowpass(angular_velocity_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);


f = figure('Name',"SysID Low Pass Filter IP");
plot(time,input)
title("Input vs. Time")
xlabel("Time [s]")
ylabel("Normalized Input")
grid minor
saveas(f, 'helicopter_sysID_plots/lowpass_IP','epsc');

f = figure('Name',"SysID Low Pass Filter Output");
subplot(2,1,1)
hold on
plot(time,pitch_)
plot(time,pitch)
title("Pitch vs. Time")
xlabel("Time [s]")
ylabel("Pitch Angle [rad]")
legend("Unfiltered","Filtered")
grid minor
subplot(2,1,2)
hold on
plot(time,angular_velocity_)
plot(time,angular_velocity)
title("Angular Velocity vs. Time")
xlabel("Time [s]")
ylabel("Angular Velocity [rpm]")
legend("Unfiltered","Filtered")
grid minor

saveas(f, 'helicopter_sysID_plots/lowpass_output','epsc');
%%


% ======================================================================= %
% -------------------- SYSTEM ID FOR POSITIVE INPUTS -------------------- %
% ======================================================================= %

pos.input = cell(1,14);
pos.output = cell(1,14);
pos.pitch_mean = zeros(1,14);
pos.angular_velocity_mean = zeros(1,14);
pos.input_mean = zeros(1,14);

disp("Identification in progress")

% Create cell array with all the input/output datasets
for i = 1:1:14

    if (i <= 10)
        % Load step response data
        load("id_data/step_pos_id_" + num2str(i) + ".mat")
    elseif (i > 10)
        % Load chirp response data
        load("id_data/chirp_pos_id_" + num2str(i-10) + ".mat")
    end

    % Read variables
    time = sensor_out.Time; % [s] Run time
    input = ScopeData1.signals(1).values; % Normalized input
    pitch_ = movmean(sensor_out.Data(:,1),3); % [radians] Pitch angle
    angular_velocity_ = sensor_out.Data(:,2); % [rpm] Angular velocity
        
    % Apply low pass filter on the variables
    pitch = lowpass(pitch_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);
    angular_velocity = lowpass(angular_velocity_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);

    pos.input_mean(i) = mean(input(time>60));
    pos.pitch_mean(i) = mean(pitch(time>60));
    pos.angular_velocity_mean(i) = mean(angular_velocity(time>60));

    % x_op1 = 0;
    % x_op2 = abs(mean(pitch(time>60)));
    % x_op3 = abs(mean(angular_velocity(time>60)));
    
    pos.input{i} = [zeros(sysID_par.fs,1); input];
    pos.output{i} = [zeros(sysID_par.fs,2); pitch angular_velocity];

end


% Define Grey Box ID data
helicopterSysData_pos = iddata(pos.output,pos.input,sysID_par.Ts);
set(helicopterSysData_pos,'OutputName',{'Pitch Angle', 'Angular Velocity'},'OutputUnit',{'rad';'rpm'}, ...
                      'InputName','V_ele');

% Define Grey Box ID state space parameters
helicopterSysGrey_pos = idgrey(@helicopterSys,{'a1',0.14;'a2',10;'a3',0.006;'a4',2.6; 'b',1000},'c');

% Run Grey Box ID algorithm
opt_greyest = greyestOptions('InitialState','zero');
helicopterSysID_pos = greyest(helicopterSysData_pos,helicopterSysGrey_pos,opt_greyest)
disp("Identification completed for positive inputs")

% Load step response data
load("id_data/step_pos_id_1.mat")

% Read variables
time = sensor_out.Time; % [s] Run time

pos.pitch_offset = zeros(1,10);
pos.angular_velocity_offset = zeros(1,10);
pos.mean.pitch = pos.pitch_mean(1:10);
pos.mean.angular_velocity = pos.angular_velocity_mean(1:10);

for i = 1:1:10

    input = pos.input{i};
    input = input(1001:end);
    output = lsim(helicopterSysID_pos,input,time);
    input = output(1001:end,:);

    pos.pitch_offset(i) = pos.pitch_mean(i)-mean(output(time>60,1));

    pos.angular_velocity_offset(i) = pos.angular_velocity_mean(i)-mean(output(time>60,2));

end


%%
% ======================================================================= %
% -------------------- SYSTEM ID FOR NEGATIVE INPUTS -------------------- %
% ======================================================================= %

neg.input = cell(1,14);
neg.output = cell(1,14);
neg.pitch_mean = zeros(1,14);
neg.angular_velocity_mean = zeros(1,14);
neg.input_mean = zeros(1,14);

disp("Identification in progress")

% Create cell array with all the input/output datasets
for i = 1:1:14

    if (i <= 10)
        % Load step response data
        load("id_data/step_neg_id_" + num2str(i) + ".mat")
    elseif (i > 10)
        % Load chirp response data
        load("id_data/chirp_neg_id_" + num2str(i-10) + ".mat")
    end

    % Read variables
    time = sensor_out.Time; % [s] Run time    
    input = ScopeData1.signals(1).values; % Normalized input
    pitch_ = movmean(sensor_out.Data(:,1),3); % [radians] Pitch angle
    angular_velocity_ = sensor_out.Data(:,2); % [rpm] Angular velocity
    
    % Apply low pass filter on the variables
    pitch = lowpass(pitch_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);
    angular_velocity = lowpass(angular_velocity_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);

    neg.input_mean(i) = mean(input(time>60));
    neg.pitch_mean(i) = mean(pitch(time>60));
    neg.angular_velocity_mean(i) = mean(angular_velocity(time>60));
   
    neg.input{i} = [zeros(sysID_par.fs,1); input];
    neg.output{i} = [zeros(sysID_par.fs,2); pitch angular_velocity];
end

% Define Grey Box ID data
helicopterSysData_neg = iddata(neg.output,neg.input,sysID_par.Ts);
set(helicopterSysData_neg,'OutputName',{'Pitch Angle', 'Angular Velocity'},'OutputUnit',{'rad';'rpm'}, ...
                      'InputName','V_ele');

% Define Grey Box ID state space parameters
helicopterSysGrey_neg = idgrey(@helicopterSys,{'a1',0.2;'a2',9;'a3',0.012;'a4',2.9;'b',1100},'c');

% Run Grey Box ID algorithm
opt_greyest = greyestOptions('InitialState','zero');
helicopterSysID_neg = greyest(helicopterSysData_neg,helicopterSysGrey_neg,opt_greyest)
disp("Identification completed for negative inputs")


% Load step response data
load("id_data/step_neg_id_1.mat")

% Read variables
time = sensor_out.Time; % [s] Run time

neg.pitch_offset = zeros(1,10);
neg.angular_velocity_offset = zeros(1,10);

neg.mean.pitch = neg.pitch_mean(1:10);
neg.mean.angular_velocity = neg.angular_velocity_mean(1:10);

for i = 1:1:10

    input = neg.input{i};
    input = input(1001:end);
    output = lsim(helicopterSysID_neg,input,time);
    input = output(1001:end,:);

    neg.pitch_offset(11-i) = neg.pitch_mean(i)-mean(output(time>60,1));
    neg.angular_velocity_offset(11-i) = neg.angular_velocity_mean(i)-mean(output(time>60,2));

end

%%
% ======================================================================= %
% ----------------- VALIDATION FOR THE POSITIVE MODEL ------------------- %
% ======================================================================= %

pos.fit.pitch_nrmse = zeros(1,4);
pos.fit.pitch_nmse = zeros(1,4);
pos.fit.angVel_nrmse = zeros(1,4);
pos.fit.angVel_nmse = zeros(1,4);

% Create cell array with all the input/output datasets
for i = 1:1:4

    % Load response data
    load("validation_data/chirp_pos_val_" + num2str(i) + ".mat")

    % Read variables
    time = sensor_out.Time(2:end); % [s] Run time
    time(1) = 0;
    input = ScopeData1.signals(1).values(2:end); % Normalized input
    pitch_ = movmean(sensor_out.Data(2:end,1),3); % [radians] Pitch angle
    angular_velocity_ = sensor_out.Data(2:end,2); % [rpm] Angular velocity

    % Apply low pass filter on the variables
    pitch = lowpass(pitch_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);
    angular_velocity = lowpass(angular_velocity_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);
    
    validation_input = [time input];
    simulation_time = time(end);
    sim("helicopter_sysID_sim_21b.slx",simulation_time);
    validation_output = simulation_output.Data;

    pos.fit.pitch_nrmse(i) = (1-goodnessOfFit(validation_output(:,1),pitch,'NRMSE'))*100;
    pos.fit.pitch_nmse(i) = (1-goodnessOfFit(validation_output(:,1),pitch,'NMSE'))*100;

    pos.fit.angVel_nrmse(i) = (1-goodnessOfFit(validation_output(:,2),angular_velocity,'NRMSE'))*100;
    pos.fit.angVel_nmse(i) = (1-goodnessOfFit(validation_output(:,2),angular_velocity,'NMSE'))*100;

end


% Display NRMSE and NMSE for the positive system
disp("POSITIVE: Fit with NRMSE")
disp(['Pitch: ',num2str(mean(pos.fit.pitch_nrmse),4),'%']);
disp(['Angular Velocity: ',num2str(mean(pos.fit.angVel_nrmse),4),'%']);
disp(' ')
disp("POSITIVE: Fit with NMSE")
disp(['Pitch: ', num2str(mean(pos.fit.pitch_nmse),4),'%']);
disp(['Angular Velocity: ', num2str(mean(pos.fit.angVel_nmse),4),'%']);


f = figure('Name',"Positive Validation IP");
hold on
plot(time,input)
grid minor
xlabel("Time [s]")
ylabel("Normalized Input [.]")
title("Input vs. Time")
saveas(f, 'helicopter_sysID_plots/validation_pos_IP','epsc');

f = figure('Name',"Positive Validation");
subplot(2,1,1)
hold on
plot(time,pitch)
plot(time,validation_output(:,1))
grid minor
xlabel("Time [s]")
ylabel("Pitch Angle [rad]")
title("Pitch Angle vs. Time")
legend("Real Data","Simulation",'Location','southeast')

subplot(2,1,2)
hold on
plot(time,angular_velocity)
plot(time,validation_output(:,2))
grid minor
xlabel("Time [s]")
ylabel("Angular Velocity [deg/s]")
title("Angular Velocity vs. Time")
legend("Real Data","Simulation",'Location','southeast')

saveas(f, 'helicopter_sysID_plots/validation_pos','epsc');

%%
% ======================================================================= %
% ----------------- VALIDATION FOR THE NEGATIVE MODEL ------------------- %
% ======================================================================= %

neg.fit.pitch_nrmse = zeros(1,4);
neg.fit.pitch_nmse = zeros(1,4);
neg.fit.angVel_nrmse = zeros(1,4);
neg.fit.angVel_nmse = zeros(1,4);

% Create cell array with all the input/output datasets
for i = 1:1:4

    % Load response data
    load("validation_data/chirp_neg_val_" + num2str(i) + ".mat")

    % Read variables
    time = sensor_out.Time(2:end); % [s] Run time
    time(1) = 0;
    input = ScopeData1.signals(1).values(2:end); % Normalized input
    pitch_ = movmean(sensor_out.Data(2:end,1),3); % [radians] Pitch angle
    angular_velocity_ = sensor_out.Data(2:end,2); % [rpm] Angular velocity

    % Apply low pass filter on the variables
    pitch = lowpass(pitch_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);
    angular_velocity = lowpass(angular_velocity_,sysID_par.fpass,sysID_par.fs,"Steepness",0.99,"StopbandAttenuation",80);
    
    validation_input = [time input];
    simulation_time = time(end);
    sim("helicopter_sysID_sim_21b.slx",simulation_time);
    validation_output = simulation_output.Data;

    neg.fit.pitch_nrmse(i) = (1-goodnessOfFit(validation_output(:,1),pitch,'NRMSE'))*100;
    neg.fit.pitch_nmse(i) = (1-goodnessOfFit(validation_output(:,1),pitch,'NMSE'))*100;

    neg.fit.angVel_nrmse(i) = (1-goodnessOfFit(validation_output(:,2),angular_velocity,'NRMSE'))*100;
    neg.fit.angVel_nmse(i) = (1-goodnessOfFit(validation_output(:,2),angular_velocity,'NMSE'))*100;

end

% Display NRMSE and NMSE for the positive system
disp("NEGATIVE: Fit with NRMSE")
disp(['Pitch: ',num2str(mean(neg.fit.pitch_nrmse),4),'%']);
disp(['Angular Velocity: ',num2str(mean(neg.fit.angVel_nrmse),4),'%']);
disp(' ')
disp("NEGATIVE: Fit with NMSE")
disp(['Pitch: ', num2str(mean(neg.fit.pitch_nmse),4),'%']);
disp(['Angular Velocity: ', num2str(mean(neg.fit.angVel_nmse),4),'%']);


f = figure('Name',"Negative Validation IP");
hold on
plot(time,input)
grid minor
xlabel("Time [s]")
ylabel("Normalized Input [.]")
title("Input vs. Time")
saveas(f, 'helicopter_sysID_plots/validation_neg_IP','epsc');

f = figure('Name',"Negative Validation");
subplot(2,1,1)
hold on
plot(time,pitch)
plot(time,validation_output(:,1))
grid minor
xlabel("Time [s]")
ylabel("Pitch Angle [rad]")
title("Pitch Angle vs. Time")
legend("Real Data","Simulation")

subplot(2,1,2)
hold on
plot(time,angular_velocity)
plot(time,validation_output(:,2))
grid minor
xlabel("Time [s]")
ylabel("Angular Velocity [deg/s]")
title("Angular Velocity vs. Time")
legend("Real Data","Simulation")

saveas(f, 'helicopter_sysID_plots/validation_neg','epsc');


%%
% Display the identified A and B matrices
disp('A_pos:'); % sounds like blood types hahaha
disp(helicopterSysID_pos.A);
disp('B_pos:');
disp(helicopterSysID_pos.B);
disp('A_neg:');
disp(helicopterSysID_neg.A);
disp('B_neg:');
disp(helicopterSysID_neg.B);

%%


% Compare the identified system with the real data
% load("id_data/step_id_21.mat")
load("id_data/chirp_02.mat")
    
% Read variables
time = sensor_out.Time(1:end); % [s] Runtime
input = ScopeData1.signals(1).values(1:end); % Normalized input
pitch = movmean(sensor_out.Data(1:end,1),3); % [radians] Pitch angle
angular_velocity = sensor_out.Data(1:end,2); % [rpm] Angular velocity

validation_input = [time input];
simulation_time = time(end);
sim("helicopter_sysID_sim_21b.slx",simulation_time)

disp("Fit with NRMSE: ")
fit.pitch_nrmse = (1-goodnessOfFit(simulation_output.Data(:,1),pitch,'NRMSE'))*100;
fit.angVel_nrmse = (1-goodnessOfFit(simulation_output.Data(:,2),angular_velocity,'NRMSE'))*100;
disp(['Pitch: ',num2str(fit.pitch_nrmse,4),'%']);
disp(['Angular Velocity: ',num2str(fit.angVel_nrmse,4),'%']);
disp(' ')
disp("Fit with NMSE: ")
fit.pitch_nmse = (1-goodnessOfFit(simulation_output.Data(:,1),pitch,'NMSE'))*100;
fit.angVel_nmse = (1-goodnessOfFit(simulation_output.Data(:,2),angular_velocity,'NMSE'))*100;
disp(['Pitch: ',num2str(fit.pitch_nmse,4),'%']);
disp(['Angular Velocity: ',num2str(fit.angVel_nmse,4),'%']);

f = figure('Name',"Validation");
subplot(2,2,1:2)
hold on
plot(time,input)
grid minor
xlabel("Time [s]")
ylabel("Normalized Input [.]")
title("Input vs. Time")

subplot(2,2,3)
hold on
plot(time,pitch)
plot(simulation_output.Time,simulation_output.Data(:,1))
grid minor
xlabel("Time [s]")
ylabel("Pitch Angle [rad]")
title("Pitch Angle vs. Time")
legend("Real Data","Simulation")

subplot(2,2,4)
hold on
plot(time,angular_velocity)
plot(simulation_output.Time,simulation_output.Data(:,2))
grid minor
xlabel("Time [s]")
ylabel("Angular Velocity [deg/s]")
title("Angular Velocity vs. Time")
legend("Real Data","Simulation")

saveas(f, 'helicopter_sysID_plots/validation','epsc');

pos.a1 = helicopterSysID_pos.Report.Parameters.ParVector(1);
pos.a2 = helicopterSysID_pos.Report.Parameters.ParVector(2);
pos.a3 = helicopterSysID_pos.Report.Parameters.ParVector(3);
pos.a4 = helicopterSysID_pos.Report.Parameters.ParVector(4);
pos.b  = helicopterSysID_pos.Report.Parameters.ParVector(5);

neg.a1 = helicopterSysID_neg.Report.Parameters.ParVector(1);
neg.a2 = helicopterSysID_neg.Report.Parameters.ParVector(2);
neg.a3 = helicopterSysID_neg.Report.Parameters.ParVector(3);
neg.a4 = helicopterSysID_neg.Report.Parameters.ParVector(4);
neg.b  = helicopterSysID_neg.Report.Parameters.ParVector(5);

save("sysID_output.mat","helicopterSysID_pos","helicopterSysID_neg","pos","neg")

% State-space representation of the linearized dynamical system
% function [A,B,C,D] = helicopterSys(a1, a2, a3, a4, b, x_op2, x_op3, Ts)
% 
%     A = [-a1 -a2*cos(x_op2) 2*a3*x_op3;
%          1 0 0;
%          0 0 -2*a4*x_op3];
%     B = [0 ; 0 ; b];
%     C = [0 1 0;
%          0 0 1];
%     D = [0;0];
% 
% end

function [A,B,C,D] = helicopterSys(a1, a2, a3, a4, b, Ts)

    A = [-a1 -a2 a3;
         1 0 0;
         0 0 -a4];
    B = [0 ; 0 ; b];
    C = [0 1 0;
         0 0 1];
    D = [0;0];

end
