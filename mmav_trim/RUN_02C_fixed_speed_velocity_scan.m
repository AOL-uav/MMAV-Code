%% RUN_02C_FIXED_SPEED_VELOCITY_SCAN
% Fixed-speed force-balance scan for the same shared configuration.
%
% This is not the full passive trim solve.  It fixes V, solves alpha/gamma
% only from the two force-balance equations, and reports the remaining Cm_y.
% A passive trimmed speed is indicated where Cm_y crosses zero while the
% force-balance residual is small and dCm/dalpha is acceptable.

clear; clc; close all;
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(project_dir));

fprintf('\n============================================================\n');
fprintf('RUN 02C: fixed-speed velocity scan / force-balance diagnostic\n');
fprintf('============================================================\n');

U = MMAV_USER_CONFIG();
[cfg, vlm_cfg, layout] = mmav_configure_case(U);
polar_info = mmav_print_vlm_polar_info(vlm_cfg);
gap = mmav_tail_wing_gap_info(cfg);

active_cg_B_mm = U.active_cg_B_mm;
if isfield(U.eom,'velocity_sweep_V_mps') && ~isempty(U.eom.velocity_sweep_V_mps)
    V_vec = reshape(U.eom.velocity_sweep_V_mps, [], 1);
else
    V_vec = (4.0:0.25:10.0).';
end

bounds2.lb = [U.eom.bounds.lb(2), U.eom.bounds.lb(3)];
bounds2.ub = [U.eom.bounds.ub(2), U.eom.bounds.ub(3)];
options = U.eom.solver_options;

fprintf('\nShared config file : MMAV_USER_CONFIG.m\n');
fprintf('Tail aero mode     : %s\n', cfg.curled_tail.aero_mode);
fprintf('Active CG_B        : [%.2f %.2f %.2f] mm\n', active_cg_B_mm);
fprintf('Frame reference    : opt0615 selected CG_B %.2f mm; wing root LE_B %.2f mm\n', layout.cg_B_mm, layout.wing_rootLE_B_mm);
fprintf('Tail design        : %s\n', cfg.curled_tail.design_note);
fprintf('Tail clearance each side = %.2f mm, overlap = %.2f mm\n', gap.clearance_each_side_mm, gap.overlap_each_side_mm);
fprintf('Velocity scan      : %.2f to %.2f m/s, N=%d\n', min(V_vec), max(V_vec), numel(V_vec));

Rows = table();
x0 = [U.eom.z0(2), U.eom.z0(3)];
for ii = 1:numel(V_vec)
    V = V_vec(ii);
    fprintf('  V = %.2f m/s...\n', V);
    try
        fb = mmav_solve_force_balance_at_speed(cfg, vlm_cfg, active_cg_B_mm, V, x0, bounds2, options);
        if isfinite(fb.force_residual_norm) && fb.force_residual_norm < 0.05
            x0 = [fb.alpha_deg, fb.gamma_deg];
        end
    catch ME
        warning('Fixed-speed solve failed at V=%.2f: %s', V, ME.message);
        fb = empty_fixed_speed_result(V);
    end

    pass_speed_point = fb.force_balance_ok && abs(fb.Cm_y) <= U.screen_eom.max_abs_Cm && ...
        fb.dCm_dalpha_per_rad <= U.screen_eom.max_dCm_dalpha && ...
        fb.detail.LD >= U.screen_eom.min_LD && ...
        gap.overlap_each_side_mm <= U.screen_eom.max_tail_overlap_mm && fb.vlm_converged;

    row = table(V, fb.alpha_deg, fb.gamma_deg, fb.theta_deg, fb.residual(1), fb.residual(2), fb.Cm_y, ...
        fb.force_residual_norm, fb.dCm_dalpha_per_rad, fb.detail.CL_wind, fb.detail.CD_wind, fb.detail.LD, ...
        fb.detail.L_over_W, fb.detail.D_over_W, fb.vlm_converged, fb.detail.solver_err, fb.exitflag, pass_speed_point, ...
        'VariableNames', {'V_mps','alpha_deg','gamma_deg','theta_deg','R_lift','R_drag','Cm_y', ...
        'force_residual_norm','dCm_dalpha','CL','CD','LD','L_over_W','D_over_W','vlm_converged','vlm_solver_err','exitflag','pass_speed_point'});
    Rows = [Rows; row]; %#ok<AGROW>
end

% Estimate passive trim speed where Cm crosses zero on force-balanced branch.
valid = isfinite(Rows.Cm_y) & isfinite(Rows.V_mps) & Rows.force_residual_norm < 0.05;
V_Cm0 = NaN;
if nnz(valid) >= 2
    V_Cm0 = mmav_find_crossing(Rows.V_mps(valid), Rows.Cm_y(valid), 0);
end

fprintf('\n--- Fixed-speed force-balance scan table ---\n');
disp(Rows);

fprintf('\nInterpretation:\n');
fprintf('  This scan fixes V and solves only alpha/gamma from L and D balance.\n');
fprintf('  Cm_y is the remaining passive pitch-trim residual at that speed.\n');
if isfinite(V_Cm0)
    fprintf('  Estimated speed where Cm_y=0 on this branch: %.3f m/s\n', V_Cm0);
else
    fprintf('  No Cm_y=0 crossing found in the scanned speed range.\n');
end
fprintf('  Full EOM trim should agree with this crossing only if force residuals are small.\n');


%% Auto-save result and settings
results_dir = fullfile(project_dir, 'results', 'velocity_scan');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

settings = struct();
settings.created_datetime = datestr(now, 'yyyy-mm-dd HH:MM:SS');
settings.script_name = mfilename;
settings.config_file = 'MMAV_USER_CONFIG.m';
settings.velocity_sweep_V_mps = V_vec;
settings.V_min_mps = min(V_vec);
settings.V_max_mps = max(V_vec);
settings.N_speed = numel(V_vec);
if numel(V_vec) >= 2
    settings.V_step_mps = median(diff(V_vec));
else
    settings.V_step_mps = NaN;
end
settings.active_cg_B_mm = active_cg_B_mm;
settings.tail_aero_mode = cfg.curled_tail.aero_mode;
settings.tail_origin_B_mm = cfg.curled_tail.origin_B_mm;
settings.tail_span_mm = cfg.curled_tail.span_mm;
settings.tail_arc_chord_mm = cfg.curled_tail.arc_chord_mm;
settings.tail_curl_deg = cfg.curled_tail.curl_deg;
settings.tail_incidence_deg = cfg.curled_tail.incidence_deg;
settings.tail_drag_CD = local_getfield(cfg.curled_tail, 'drag_CD', NaN);
settings.tail_drag_scale = local_getfield(cfg.curled_tail, 'drag_scale', NaN);
settings.tail_gap = gap;
settings.layout = layout;
settings.bounds_alpha_gamma = bounds2;
settings.solver_options = options;
settings.V_Cm0 = V_Cm0;
settings.polar_info = polar_info;
settings.note = 'Rows contains the fixed-speed force-balance scan; settings records the exact configuration used.';

V_tag = sprintf('V%s_to_%s_dV%s_N%d', ...
    local_numtag(settings.V_min_mps, '%.2f'), ...
    local_numtag(settings.V_max_mps, '%.2f'), ...
    local_numtag(settings.V_step_mps, '%.2f'), ...
    settings.N_speed);
CG_tag = sprintf('CGx%s_y%s_z%s', ...
    local_numtag(active_cg_B_mm(1), '%.1f'), ...
    local_numtag(active_cg_B_mm(2), '%.1f'), ...
    local_numtag(active_cg_B_mm(3), '%.1f'));
Tail_tag = sprintf('tail_%s_span%s_arc%s_curl%s_inc%s', ...
    local_clean_tag(cfg.curled_tail.aero_mode), ...
    local_numtag(cfg.curled_tail.span_mm, '%.1f'), ...
    local_numtag(cfg.curled_tail.arc_chord_mm, '%.1f'), ...
    local_numtag(cfg.curled_tail.curl_deg, '%.1f'), ...
    local_numtag(cfg.curled_tail.incidence_deg, '%.1f'));
Time_tag = datestr(now, 'yyyymmdd_HHMMSS');

result_filename = sprintf('RUN02C_%s_%s_%s_%s.mat', V_tag, CG_tag, Tail_tag, Time_tag);
result_filename = local_clean_filename(result_filename);
result_path = fullfile(results_dir, result_filename);

save(result_path, 'Rows', 'settings', 'U', 'cfg', 'layout', 'gap', 'V_Cm0');

csv_path = strrep(result_path, '.mat', '.csv');
writetable(Rows, csv_path);

fprintf('\n[SAVE-DIAG] Saved MAT: %s\n', result_path);
fprintf('[SAVE-DIAG] Saved CSV: %s\n', csv_path);

figure('Name','RUN 02C: fixed-speed velocity scan','Color','w');
tiledlayout(2,2,'TileSpacing','compact');
nexttile;
plot(Rows.V_mps, Rows.alpha_deg, 'o-', 'LineWidth',1.2); grid on;
xlabel('V [m/s]'); ylabel('\alpha  [deg]');
nexttile;
plot(Rows.V_mps, Rows.gamma_deg, 's-', 'LineWidth',1.2); grid on;
xlabel('V [m/s]'); ylabel('\gamma  [deg]');
nexttile;
plot(Rows.V_mps, Rows.Cm_y, 'o-', 'LineWidth',1.2); hold on; yline(0,'k--');
if isfinite(V_Cm0), xline(V_Cm0,':',sprintf('Cm=0 %.2f m/s',V_Cm0)); end
grid on; xlabel('V [m/s]'); ylabel('C_{m,y}');
nexttile;
plot(Rows.V_mps, Rows.LD, 'o-', 'LineWidth',1.2); grid on;
xlabel('V [m/s]'); ylabel('L/D');

function fb = empty_fixed_speed_result(V)
fb = struct();
fb.V_mps = V;
fb.alpha_deg = NaN;
fb.gamma_deg = NaN;
fb.theta_deg = NaN;
fb.residual = [Inf; Inf; Inf];
fb.force_residual_norm = Inf;
fb.Cm_y = NaN;
fb.dCm_dalpha_per_rad = NaN;
fb.vlm_converged = false;
fb.force_balance_ok = false;
fb.exitflag = NaN;
fb.detail = struct('CL_wind',NaN,'CD_wind',NaN,'LD',NaN,'L_over_W',NaN,'D_over_W',NaN,'solver_err',NaN);
end


function s = local_numtag(x, fmt)
if nargin < 2 || isempty(fmt)
    fmt = '%.2f';
end
if ~isfinite(x)
    s = 'NaN';
    return;
end
s = sprintf(fmt, x);
s = strrep(s, '-', 'm');
s = strrep(s, '+', '');
s = strrep(s, '.', 'p');
end

function s = local_clean_tag(x)
s = char(string(x));
s = regexprep(s, '[^A-Za-z0-9_\-]+', '_');
s = regexprep(s, '_+', '_');
s = regexprep(s, '^_|_$', '');
if isempty(s)
    s = 'none';
end
end

function fname = local_clean_filename(fname)
fname = char(string(fname));
fname = regexprep(fname, '[<>:"/\|?*]+', '_');
fname = regexprep(fname, '\s+', '_');
end

function val = local_getfield(S, fieldname, default_val)
if isstruct(S) && isfield(S, fieldname)
    val = S.(fieldname);
else
    val = default_val;
end
end
