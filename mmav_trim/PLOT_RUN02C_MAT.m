%% PLOT_RUN02C_MAT
% Simple plot script for RUN_02C velocity-scan .mat files.
% Edit the mat_file path below as needed.

clear; clc; close all;

% mat_file = '/mnt/data/vel_sweep_5to10.mat';
mat_file = 'vel_sweep_5to10.mat';

S = load(mat_file);
if ~isfield(S,'Rows')
    error('The MAT file does not contain variable "Rows".');
end
Rows = S.Rows;

V = Rows.V_mps;

%% 1) alpha, gamma, theta
figure('Color','w');
subplot(3,1,1);
plot(V, Rows.alpha_deg, 'o-', 'LineWidth', 1.2);
ylabel('\alpha [deg]');
grid on;

subplot(3,1,2);
plot(V, Rows.gamma_deg, 'o-', 'LineWidth', 1.2);
ylabel('\gamma [deg]');
grid on;

subplot(3,1,3);
plot(V, Rows.theta_deg, 'o-', 'LineWidth', 1.2);
ylabel('\theta [deg]');
xlabel('V [m/s]');
grid on;

%% 2) force residuals
figure('Color','w');
subplot(3,1,1);
plot(V, Rows.R_lift, 'o-', 'LineWidth', 1.2);
ylabel('R_{lift}');
grid on;

subplot(3,1,2);
plot(V, Rows.R_drag, 'o-', 'LineWidth', 1.2);
ylabel('R_{drag}');
grid on;

subplot(3,1,3);
plot(V, Rows.force_residual_norm, 'o-', 'LineWidth', 1.2);
ylabel('||R||');
xlabel('V [m/s]');
grid on;

%% 3) CL, CD, L/D
figure('Color','w');
subplot(3,1,1);
plot(V, Rows.CL, 'o-', 'LineWidth', 1.2);
ylabel('C_L');
grid on;

subplot(3,1,2);
plot(V, Rows.CD, 'o-', 'LineWidth', 1.2);
ylabel('C_D');
grid on;

subplot(3,1,3);
plot(V, Rows.LD, 'o-', 'LineWidth', 1.2);
ylabel('L/D');
xlabel('V [m/s]');
grid on;
