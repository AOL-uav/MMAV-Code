%% RUN_02B_EOM_GLIDE_TRIM
% Nonlinear algebraic EOM trim solve for unpowered steady glide.
%
% This solves z = [V, alpha, gamma], rather than imposing alpha from a sweep:
%   L(V,alpha) - W*cos(gamma) = 0
%   D(V,alpha) - W*sin(gamma) = 0
%   Cm_y(V,alpha)             = 0
% gamma is positive downward.  The corresponding conventional longitudinal
% state is u = V*cos(alpha), w = V*sin(alpha), theta = alpha - gamma.

clear; clc; close all;
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(project_dir));

fprintf('\n============================================================\n');
fprintf('RUN 02B: EOM-based steady-glide trim solve\n');
fprintf('============================================================\n');

U = MMAV_USER_CONFIG();
[cfg, vlm_cfg, layout] = mmav_configure_case(U);
mmav_print_vlm_polar_info(vlm_cfg);
vehicle = mmav_build_coarse_vehicle(cfg);
gap = mmav_tail_wing_gap_info(cfg);

active_cg_B_mm = U.active_cg_B_mm;
z0 = U.eom.z0;
bounds = U.eom.bounds;
solver_options = U.eom.solver_options;

fprintf('\nShared config file : MMAV_USER_CONFIG.m\n');
fprintf('Tail aero mode     : %s\n', cfg.curled_tail.aero_mode);
fprintf('Active CG_B        : [%.2f %.2f %.2f] mm\n', active_cg_B_mm);
fprintf('Opt0615 selected CG_B = %.2f mm; wing root LE_B = %.2f mm\n', layout.cg_B_mm, layout.wing_rootLE_B_mm);
fprintf('Tail design        : %s\n', cfg.curled_tail.design_note);
fprintf('Tail actual area scale relative to original = %.3f\n', cfg.curled_tail.actual_area_scale_relative_to_original);
fprintf('Tail clearance each side = %.2f mm, overlap = %.2f mm\n', gap.clearance_each_side_mm, gap.overlap_each_side_mm);
fprintf('Initial guess z0 = [V %.2f, alpha %.2f, gamma %.2f]\n', z0);
fprintf('Velocity is a solved unknown here, not swept. Use RUN_02C for fixed-speed scan.\n');

trim = mmav_solve_steady_glide_trim(cfg, vlm_cfg, active_cg_B_mm, z0, bounds, solver_options);

fprintf('\n--- EOM trim solution summary ---\n');
disp(trim.summary_table);

fprintf('\nRaw residual vector [lift/W, drag/W, Cm_y]:\n');
fprintf('  [% .6e  % .6e  % .6e], norm = %.6e\n', trim.residual(1), trim.residual(2), trim.residual(3), trim.residual_norm);
fprintf('EOM residual OK flag: %d; optimizer exitflag: %d; VLM converged: %d\n', trim.eom_residual_ok, trim.exitflag, trim.vlm_converged);
if abs(trim.V_mps - bounds.ub(1)) < 0.15
    fprintf('Note: V is close to the upper bound. Consider increasing U.eom.bounds.ub(1) or checking RUN_02C.\n');
end

fprintf('\nConventional longitudinal state interpretation:\n');
fprintf('  u_forward = %.4f m/s\n', trim.u_forward_mps);
fprintf('  w_down    = %.4f m/s\n', trim.w_down_mps);
fprintf('  theta     = %.4f deg  (= alpha - gamma, gamma positive downward)\n', trim.theta_deg);
fprintf('  beta=0, phi=0, p=q=r=0 for this symmetric steady-glide trim.\n');

fprintf('\n--- Load source table at solved trim ---\n');
disp(trim.detail.source_table);

alpha_vec = linspace(max(bounds.lb(2),trim.alpha_deg-4), min(bounds.ub(2),trim.alpha_deg+4), 21).';
L_over_W = NaN(size(alpha_vec));
D_over_W = NaN(size(alpha_vec));
Cm_vec = NaN(size(alpha_vec));
for ii = 1:numel(alpha_vec)
    [~, di] = mmav_steady_glide_trim_residual([trim.V_mps, alpha_vec(ii), trim.gamma_deg], cfg, vlm_cfg, active_cg_B_mm, U.rho_kgpm3);
    L_over_W(ii) = di.L_over_W;
    D_over_W(ii) = di.D_over_W;
    Cm_vec(ii) = di.Cm_y;
end

figure('Name','RUN 02B: EOM trim local alpha diagnostic','Color','w');
tiledlayout(1,3,'TileSpacing','compact');
nexttile;
plot(alpha_vec, L_over_W, 'o-', 'LineWidth',1.2); hold on;
yline(cosd(trim.gamma_deg),'k--','cos gamma'); xline(trim.alpha_deg,':','trim alpha'); grid on;
xlabel('\alpha [deg]'); ylabel('L/W'); title('Normal-force balance');
nexttile;
plot(alpha_vec, D_over_W, 's-', 'LineWidth',1.2); hold on;
yline(sind(trim.gamma_deg),'k--','sin gamma'); xline(trim.alpha_deg,':','trim alpha'); grid on;
xlabel('\alpha [deg]'); ylabel('D/W'); title('Path-axis force balance');
nexttile;
plot(alpha_vec, Cm_vec, 'o-', 'LineWidth',1.2); hold on;
yline(0,'k--'); xline(trim.alpha_deg,':','trim alpha'); grid on;
xlabel('\alpha [deg]'); ylabel('C_{m,y}'); title('Pitch trim');
