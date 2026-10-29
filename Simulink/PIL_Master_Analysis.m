% --- MASTER RUNNER & METRICS EVALUATOR ---
clear; clc;

modelName = 'Processing_In_The_Loop';

% 1. Run the Simulink Model Programmatically
fprintf('Running Simulink simulation...\n');
simOut = sim(modelName);

%% 2. Extract Logged Data from Scope Workspace Logging
% Pitch PID
Pitch_PID_timeData = simOut.Pitch_Data_PID.getElement(1).Values.Time;
Pitch_PID_mtx      = simOut.Pitch_Data_PID.getElement(1).Values.Data;
Pitch_PID_ref      = Pitch_PID_mtx(:, 1);
Pitch_PID_est      = Pitch_PID_mtx(:, 2);

% Roll PID
Roll_PID_timeData  = simOut.Roll_Data_PID.getElement(1).Values.Time;
Roll_PID_mtx       = simOut.Roll_Data_PID.getElement(1).Values.Data;
Roll_PID_ref       = Roll_PID_mtx(:, 1);
Roll_PID_est       = Roll_PID_mtx(:, 2);

% Pitch Lead-Lag
Pitch_LL_timeData  = simOut.Pitch_Data_LL.getElement(1).Values.Time;
Pitch_LL_mtx       = simOut.Pitch_Data_LL.getElement(1).Values.Data;
Pitch_LL_ref       = Pitch_LL_mtx(:, 1);
Pitch_LL_est       = Pitch_LL_mtx(:, 2);

% Roll Lead-Lag
Roll_LL_timeData   = simOut.Roll_Data_LL.getElement(1).Values.Time;
Roll_LL_mtx        = simOut.Roll_Data_LL.getElement(1).Values.Data;
Roll_LL_ref        = Roll_LL_mtx(:, 1);
Roll_LL_est        = Roll_LL_mtx(:, 2);

%% 3. Extract Control Effort (Elevator & Aileron Deflections)
% Pitch PID Actuator (Elevator)
elev_PID_mtx = simOut.Pitch_Data_PID.getElement(2).Values.Data;
elev_PID_val = elev_PID_mtx(:, 1);

% Pitch Lead-Lag Actuator (Elevator)
elev_LL_mtx  = simOut.Pitch_Data_LL.getElement(2).Values.Data;
elev_LL_val  = elev_LL_mtx(:, 1);

% Roll PID Actuator (Aileron)
ail_PID_mtx  = simOut.Roll_Data_PID.getElement(2).Values.Data;
ail_PID_val  = ail_PID_mtx(:, 1);

% Roll Lead-Lag Actuator (Aileron)
ail_LL_mtx   = simOut.Roll_Data_LL.getElement(2).Values.Data;
ail_LL_val   = ail_LL_mtx(:, 1);

%% 4. Compute Tracking RMSE (Root Mean Square Error)
pitch_PID_rmse = sqrt(mean((Pitch_PID_ref - Pitch_PID_est).^2));
pitch_LL_rmse  = sqrt(mean((Pitch_LL_ref  - Pitch_LL_est).^2));

roll_PID_rmse  = sqrt(mean((Roll_PID_ref  - Roll_PID_est).^2));
roll_LL_rmse   = sqrt(mean((Roll_LL_ref   - Roll_LL_est).^2));

%% 5. Compute Control Effort Variance (Measures Surface Chatter/Jitter)
pitch_PID_var  = var(elev_PID_val);
pitch_LL_var   = var(elev_LL_val);

roll_PID_var   = var(ail_PID_val);
roll_LL_var    = var(ail_LL_val);

%% 6. --- DISPLAY COMPARATIVE RESULTS ---
fprintf('\n==================== EVALUATION METRICS ====================\n');
fprintf('AXIS     CONTROLLER   TRACKING RMSE (deg)   CONTROL VAR (deg^2)\n');
fprintf('------------------------------------------------------------\n');
fprintf('Pitch    PID          %18.4f   %18.6f\n', pitch_PID_rmse, pitch_PID_var);
fprintf('Pitch    Lead-Lag     %18.4f   %18.6f\n', pitch_LL_rmse,  pitch_LL_var);
fprintf('------------------------------------------------------------\n');
fprintf('Roll     PID          %18.4f   %18.6f\n', roll_PID_rmse,  roll_PID_var);
fprintf('Roll     Lead-Lag     %18.4f   %18.6f\n', roll_LL_rmse,   roll_LL_var);
fprintf('============================================================\n');