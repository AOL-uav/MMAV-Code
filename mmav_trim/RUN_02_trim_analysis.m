%% RUN_02_TRIM_ANALYSIS
% Fixed-speed alpha-sweep diagnostic for one chosen CG and tail geometry.
%
% This is not the full EOM trim solve.  It is retained as a diagnostic to see
% how force balance and pitch moment vary with alpha at the selected speed.
% RUN_02B_eom_glide_trim.m is the true nonlinear algebraic trim solve.

clear; clc; close all;
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(project_dir));

fprintf('\n============================================================\n');
fprintf('RUN 02: fixed-speed VLM/drag alpha-sweep trim diagnostic\n');
fprintf('============================================================\n');

U = MMAV_USER_CONFIG();
[cfg, vlm_cfg, layout] = mmav_configure_case(U);
mmav_print_vlm_polar_info(vlm_cfg);
vehicle = mmav_build_coarse_vehicle(cfg);
gap = mmav_tail_wing_gap_info(cfg);

V_mps = U.V_mps;
rho_kgpm3 = U.rho_kgpm3;
alpha_vec_deg = U.alpha_vec_deg;
active_cg_B_mm = U.active_cg_B_mm;

fprintf('\nShared config file : MMAV_USER_CONFIG.m\n');
fprintf('Tail aero mode     : %s\n', cfg.curled_tail.aero_mode);
fprintf('Active CG_B        : [%.2f %.2f %.2f] mm\n', active_cg_B_mm);
fprintf('Opt0615 selected CG_B = %.2f mm; wing root LE_B = %.2f mm\n', layout.cg_B_mm, layout.wing_rootLE_B_mm);
fprintf('Tail design        : %s\n', cfg.curled_tail.design_note);
fprintf('Tail area scale    : %.3f\n', cfg.curled_tail.actual_area_scale_relative_to_original);
fprintf('Tail clearance     : %.2f mm, overlap = %.2f mm\n', gap.clearance_each_side_mm, gap.overlap_each_side_mm);

metrics = mmav_vlm_passive_trim_metrics(cfg, vlm_cfg, active_cg_B_mm, V_mps, rho_kgpm3, alpha_vec_deg);

Summary = table(V_mps, active_cg_B_mm(1), string(cfg.curled_tail.aero_mode), cfg.curled_tail.span_mm, cfg.curled_tail.arc_chord_mm, ...
    cfg.curled_tail.actual_area_scale_relative_to_original, cfg.curled_tail.curl_deg, cfg.curled_tail.incidence_deg, ...
    metrics.alpha_lift_deg, metrics.Cm_at_lift, metrics.dCm_dalpha_lift_per_rad, ...
    metrics.LD_at_lift, metrics.glide_gamma_deg, metrics.alpha_moment_deg, metrics.Fz_over_W_at_moment, ...
    mean(metrics.converged,'omitnan'), max(metrics.err,[],'omitnan'), ...
    'VariableNames', {'V_mps','cg_x_B_mm','tail_aero_mode','tail_span_mm','tail_arc_chord_mm','tail_area_scale','tail_curl_deg','tail_inc_deg', ...
    'alpha_lift_deg','Cm_at_lift','dCm_dalpha','LD_lift','gamma_deg','alpha_moment_deg','FzW_at_moment','converged_fraction','max_solver_err'});

fprintf('\n--- Passive fixed-speed diagnostic summary ---\n');
disp(Summary);

fprintf('\n--- Load source table at Fz/W=1 ---\n');
disp(metrics.source_table_at_lift);

figure('Name','RUN 02: force balance','Color','w');
plot(metrics.alpha_vec_deg, metrics.Fz_over_W, 'o-', 'LineWidth',1.2); hold on;
yline(1,'k--'); xline(metrics.alpha_lift_deg,':','F_z/W=1'); grid on;
xlabel('\alpha [deg]'); ylabel('F_z/W'); title('Fixed-speed vertical force diagnostic');

figure('Name','RUN 02: pitching moment','Color','w');
plot(metrics.alpha_vec_deg, metrics.Cm_y, 'o-', 'LineWidth',1.2); hold on;
yline(0,'k--'); xline(metrics.alpha_lift_deg,':','F_z/W=1'); grid on;
xlabel('\alpha [deg]'); ylabel('C_{m,y} about active CG'); title('Pitch moment diagnostic');

figure('Name','RUN 02: wind-axis coefficients','Color','w');
plot(metrics.alpha_vec_deg, metrics.CL_wind, 'o-', 'LineWidth',1.2, 'DisplayName','C_L'); hold on;
plot(metrics.alpha_vec_deg, metrics.CD_wind, 's-', 'LineWidth',1.2, 'DisplayName','C_D'); grid on;
xlabel('\alpha [deg]'); ylabel('coefficient'); title('Wind-axis coefficients'); legend('Location','best');
