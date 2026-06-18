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
pitchoffs = 3.682e-2;
pitchgain = 1;
yawoffs = 1.687e-2;
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

% Calculate reference for negative inputs
input_ref = -0.5;
idx = find(neg.input_mean == input_ref);
angle_ref = neg.mean.pitch(idx);
angVel_ref = neg.mean.angular_velocity(idx);



% ======================================================================= %
% ---------------------------- POSITIVE MPC ----------------------------- %
% ======================================================================= %

%% Tuning options
% Sunset place (horizon)
predHor = 5;
conHor  = predHor;

% Weights
q1 = 20000;
q2 = 1000;
q3 = 0.2;
Q = [q1 0 0;
    0 q2 0;
    0 0 q3];

R = 1e-3;

% Constraints
uMin = -1;
uMax = 1;

%% Model (file only needs 'sys')
% Helicopter State Space; x = [pitch ang. vel. Alpha_dot; Pitch ang. Alpha; 
% Motor angular velocity Omega]
sysc = ss(helicopterSysID_pos.A,helicopterSysID_pos.B,helicopterSysID_pos.C,helicopterSysID_pos.D);
Ts = 0.001;
sys = c2d(sysc,Ts);
x0 = [0; -angle_ref; -angVel_ref]; % initial states


%% Prediction matrices generation
% Predicted state sequence x(k) = stateSequence*x(k-1) + controlSequence*u(k-1)
% Normal lqr
K_lqr = dlqr(sys.A,sys.B,Q,R);

[stateDimension,stateSequence,controlSequence,Q_hat,R_hat] = objectiveParameters(sys,Q,R,predHor,K_lqr);

%% Objective function
H = round((controlSequence'*Q_hat*controlSequence + R_hat),4);
F = controlSequence'*Q_hat*stateSequence;

% Checkcheck
% Q_confirm=blkdiag(kron(eye(predHor),Q),Q_bar); 
% H_confirm=S'*Q_confirm*S+kron(eye(predHor),R);   
% F_confirm=S'*Q_confirm*T;
% disp(Q_hat == Q_confirm)
% disp(H == H_confirm)
% disp(F == F_confirm)

if issymmetric(H)
    disp('Hessian is symmetric')
else
    disp('Warning: Hessian is not symmetric due to a rounding error.')
end

% No contraints, then:
K_full = H\F;
K_mpc = K_full(1,:);

if round(K_mpc,4) == round(K_lqr,4)
    fprintf('LQR/unconstrained MPC gain K = [%.3f,%.4f,%.6f].\n',K_mpc(1),K_mpc(2),K_mpc(3))
else
    disp('LQR and unconstr. MPC gains do not match.')
end

%% mpcActiveSetSolver
% https://nl.mathworks.com/help/mpc/ug/solve-custom-mpc-quadratic-programming-problem-and-generate-code.html
% https://nl.mathworks.com/help/mpc/ref/mpcactivesetsolver.html#mw_1a565cdb-0c28-45f6-9b2b-b003faf354dd_sep_buutcsq-A

% Constraints
% -1 <= u <= 1
horizonEye = eye(predHor);
horizonOnesColumn = ones(predHor,1);
Ac = [-horizonEye;
       horizonEye];
bc = [horizonOnesColumn;
      horizonOnesColumn];

% Since Hessian is constant, L^(-1) can be calculated offline (faster)
L = chol(H,'lower');
Linv = L\eye(size(H,1));


%% Quadprog
% Constraints
lowerBound = horizonOnesColumn*uMin;
upperBound = horizonOnesColumn*uMax;

%% Set matrices
Ac_pos = Ac;
bc_pos = bc;
F_pos = F;
Linv_pos = Linv;
%%

% ======================================================================= %
% ---------------------------- NEGATIVE MPC ----------------------------- %
% ======================================================================= %

%% Tuning options
% Sunset place (horizon)
predHor = 5;
conHor  = predHor;

% Weights
q1 = 10000;
q2 = 1000;
q3 = 0.2;
Q = [q1 0 0;
    0 q2 0;
    0 0 q3];

R = 1e-3;

% Constraints
uMin = -1;
uMax = 1;

%% Model (file only needs 'sys')
% Helicopter State Space; x = [pitch ang. vel. Alpha_dot; Pitch ang. Alpha; 
% Motor angular velocity Omega]
sysc = ss(helicopterSysID_neg.A,helicopterSysID_neg.B,helicopterSysID_neg.C,helicopterSysID_neg.D);
Ts = 0.001;
sys = c2d(sysc,Ts);
x0 = [0; -angle_ref; -angVel_ref]; % initial states


%% Prediction matrices generation
% Predicted state sequence x(k) = stateSequence*x(k-1) + controlSequence*u(k-1)
% Normal lqr
K_lqr = dlqr(sys.A,sys.B,Q,R);

[stateDimension,stateSequence,controlSequence,Q_hat,R_hat] = objectiveParameters(sys,Q,R,predHor,K_lqr);

%% Objective function
H = round((controlSequence'*Q_hat*controlSequence + R_hat),4);
F = controlSequence'*Q_hat*stateSequence;

% Checkcheck
% Q_confirm=blkdiag(kron(eye(predHor),Q),Q_bar); 
% H_confirm=S'*Q_confirm*S+kron(eye(predHor),R);   
% F_confirm=S'*Q_confirm*T;
% disp(Q_hat == Q_confirm)
% disp(H == H_confirm)
% disp(F == F_confirm)

if issymmetric(H)
    disp('Hessian is symmetric')
else
    disp('Warning: Hessian is not symmetric due to a rounding error.')
end

% No contraints, then:
K_full = H\F;
K_mpc = K_full(1,:);

if round(K_mpc,4) == round(K_lqr,4)
    fprintf('LQR/unconstrained MPC gain K = [%.3f,%.4f,%.6f].\n',K_mpc(1),K_mpc(2),K_mpc(3))
else
    disp('LQR and unconstr. MPC gains do not match.')
end

%% mpcActiveSetSolver
% https://nl.mathworks.com/help/mpc/ug/solve-custom-mpc-quadratic-programming-problem-and-generate-code.html
% https://nl.mathworks.com/help/mpc/ref/mpcactivesetsolver.html#mw_1a565cdb-0c28-45f6-9b2b-b003faf354dd_sep_buutcsq-A

% Constraints
% -1 <= u <= 1
horizonEye = eye(predHor);
horizonOnesColumn = ones(predHor,1);
Ac = [-horizonEye;
       horizonEye];
bc = [horizonOnesColumn;
      horizonOnesColumn];

% Since Hessian is constant, L^(-1) can be calculated offline (faster)
L = chol(H,'lower');
Linv = L\eye(size(H,1));


%% Quadprog
% Constraints
lowerBound = horizonOnesColumn*uMin;
upperBound = horizonOnesColumn*uMax;

%% Set matrices
Ac_neg = Ac;
bc_neg = bc;
F_neg = F;
Linv_neg = Linv;

%% --------- FUNCTIONS ---------- %%
% ================================ %
%% Prediction matrices generation %%
% ================================ %
function [stateDimension,stateSequence,controlSequence,Q_hat,R_hat] = objectiveParameters(sys,Q,R,predHor,K_lqr)
% Predicted state sequence x = stateSequence*x0 + controlSequence*u

% % Calculate LQR gain for Lyapunov function using "Form linear-quadratic (LQ) state-feedback regulator with output weighting"
% Q_y = [q2 0;
%         0 0];   % 2/3 states measured outputs
% K_lqr = lqry(sys,Q_y,R);

% State dimension (yes, only heightA would be enough (square))
[heightA,widthA] = size(sys.A);

% stateSequence = [A; A*A; A^3; ...; A^N]
% stateSequence = [];   % Without preallocating
stateSequence = zeros(heightA*predHor,widthA);  % With preallocating (faster)

% controlSequence = [B 0 ... 0; AB B 0 ... 0; ...; A^(N-1)B A^(N-2)B ... B]
controlSequence = zeros(heightA*predHor,predHor);
rowUpdate = zeros(heightA,predHor);
rowUpdate(:,1) = sys.B;

% Q_hat = zeros((heightA*(predHor)),(widthA*(predHor)));      % w/o x0
Q_hat = zeros((heightA*(predHor+1)),(widthA*(predHor+1)));  % with x0

% Create prediction metrices with correct sizes (all me, no copy/pasting)
for i=1:1:predHor
    % stateSequence = [stateSequence; A^i]; % Without preallocating
    stateSequence((1+(i-1)*heightA):(i*heightA),(1:widthA)) = sys.A^i; % With preallocating
   
    controlSequence((1+(i-1)*heightA):i*heightA,:) = rowUpdate;
    rowUpdate = [sys.A^i*sys.B rowUpdate(:,1:(predHor-1))];
end
% Including initial state x0
% stateSequence = [eye(heightA); stateSequence];
% controlSequence = [zeros([1,predHor]); controlSequence];


% Q_hat with Q-block on diagonal & Q_bar as final diagonal block
if predHor >= 1
    for i=1:1:(predHor)
        Q_hat((1+(i-1)*heightA):(i*heightA),(1+(i-1)*widthA):(i*widthA)) = Q;
    end
end
% Obj. function has same quad. cost as inf. horizon quad. cost used by LQR
Q_bar = dlyap((sys.A-sys.B*K_lqr)', Q+K_lqr'*R*K_lqr);  % Lyapunov equation for stability
% Q_bar = zeros(size(heightA));                   % Option: NO TERMINAL COSTS
Q_hat((end-(heightA-1):end),(end-(widthA-1):end)) = Q_bar;

% Apparently kron is an easier option (put R matrices on diagonal)
R_hat = kron(eye(predHor),R);


% Another approach (same results)
% Prediction matrix from initial state
T=zeros(heightA*(predHor+1),heightA);
for k=0:predHor
    T(k*heightA+1:(k+1)*heightA,:)=sys.A^k;
end

% Prediction matrix from input x0
S=zeros(heightA*(predHor+1),1*(predHor));
for k=1:predHor
    for i=0:k-1
        S(k*heightA+1:(k+1)*heightA,i+1:(i+1))=sys.A^(k-1-i)*sys.B;
    end
end

% Without initial state x0
% stateSequence = T(4:end,:);
% controlSequence = S(4:end,:);

% With initial state x0
stateSequence = T;
controlSequence = S;

stateDimension = heightA;

end
