%% RUN_02D_CONTROLLED_TRIM_AT_SPEED
% Conventional fixed-speed trim with controls.
%
% Unknowns solved by this script:
%   alpha, gamma, wing collective incidence, tail curl deformation
%
% The wing incidence command is applied equally to left/right wings for this
% symmetric longitudinal trim.  Differential incidence is supported in the
% aero layer, but is held at zero here because no roll/yaw equations are
% included yet.

clear; clc; close all;
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(project_dir));

fprintf('\n============================================================\n');
fprintf('RUN 02D: controlled fixed-speed conventional trim\n');
fprintf('============================================================\n');

U = MMAV_USER_CONFIG();
[cfg, vlm_cfg, layout] = mmav_configure_case(U);
mmav_print_vlm_polar_info(vlm_cfg);
gap = mmav_tail_wing_gap_info(cfg);

active_cg_B_mm = U.active_cg_B_mm;

if isfield(U,'controlled_trim') && isfield(U.controlled_trim,'V_mps')
    V_trim_mps = U.controlled_trim.V_mps;
else
    V_trim_mps = U.V_mps;
end

if isfield(U,'controlled_trim') && isfield(U.controlled_trim,'x0')
    x0 = U.controlled_trim.x0;
else
    x0 = [U.eom.z0(2), U.eom.z0(3), 0.0, 0.0];
end

if isfield(U,'controlled_trim') && isfield(U.controlled_trim,'bounds')
    bounds = U.controlled_trim.bounds;
else
    bounds.lb = [-4.0, 0.0, -25.0, -30.0];
    bounds.ub = [14.0, 35.0,  25.0,  50.0];
end

if isfield(U,'controlled_trim') && isfield(U.controlled_trim,'solver_options')
    options = U.controlled_trim.solver_options;
else
    options = U.eom.solver_options;
end

fprintf('\nShared config file : MMAV_USER_CONFIG.m\n');
fprintf('Tail aero mode     : %s\n', cfg.curled_tail.aero_mode);
fprintf('Active CG_B        : [%.2f %.2f %.2f] mm\n', active_cg_B_mm);
fprintf('Frame reference    : opt0615 selected CG_B %.2f mm; wing root LE_B %.2f mm\n', layout.cg_B_mm, layout.wing_rootLE_B_mm);
fprintf('Tail design        : %s\n', cfg.curled_tail.design_note);
fprintf('Tail clearance each side = %.2f mm, overlap = %.2f mm\n', gap.clearance_each_side_mm, gap.overlap_each_side_mm);
fprintf('Fixed trim speed   : %.3f m/s\n', V_trim_mps);
fprintf('Unknown x          : [alpha gamma wing_collective tail_curl_delta]\n');
fprintf('Initial guess x0   : [%.2f %.2f %.2f %.2f]\n', x0);

trim = mmav_solve_controlled_trim_at_speed(cfg, vlm_cfg, active_cg_B_mm, V_trim_mps, x0, bounds, options);

fprintf('\n--- Controlled trim solution summary ---\n');
disp(trim.summary_table);

fprintf('\n[CTRL-TRIM final] V=%.3f alpha=%.3f gamma=%.3f theta=%.3f | wingC=%+.3f tailCurlDelta=%+.3f tailCurlTotal=%.3f | R=[%+.3e %+.3e %+.3e] norm=%.3e | Cm=%+.4f dCm/da=%+.4f | L/W=%.4f D/W=%.4f L/D=%.3f | ok=%d vlm=%d exit=%d\n', ...
    trim.V_mps, trim.alpha_deg, trim.gamma_deg, trim.theta_deg, ...
    trim.wing_collective_deg, trim.tail_curl_delta_deg, trim.tail_controlled_curl_deg, ...
    trim.residual(1), trim.residual(2), trim.residual(3), trim.residual_norm, ...
    trim.detail.Cm_y, trim.dCm_dalpha_per_rad, ...
    trim.detail.L_over_W, trim.detail.D_over_W, trim.detail.LD, ...
    trim.trim_residual_ok, trim.vlm_converged, trim.exitflag);

fprintf('\n--- Load source table at controlled trim ---\n');
disp(trim.detail.source_table);

fprintf('\nNotes:\n');
fprintf('  wing_collective changes both wing incidence angles equally.\n');
fprintf('  wing differential is available in mmav_eval_vlm_aero_state but held at zero here.\n');
fprintf('  tailCurlDelta is added to the baseline tail curl angle from MMAV_USER_CONFIG.m.\n');
