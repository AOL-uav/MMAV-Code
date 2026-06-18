%% RUN_02E_CONTROLLED_VELOCITY_SWEEP
% Fixed-speed conventional trim sweep with controls.
%
% For each prescribed V, this solves:
%   x = [alpha_deg, gamma_deg, wing_collective_deg, tail_curl_delta_deg]
%
% Residuals:
%   R_lift = L/W - cos(gamma)
%   R_drag = D/W - sin(gamma)
%   R_Cm   = Cm_y
%
% Controls:
%   wing_collective_deg : symmetric incidence command for both wings
%   tail_curl_delta_deg : deformation command added to baseline tail curl

clear; clc; close all;
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(project_dir));

fprintf('\n============================================================\n');
fprintf('RUN 02E: controlled velocity sweep / conventional trim\n');
fprintf('============================================================\n');

U = MMAV_USER_CONFIG();
[cfg, vlm_cfg, layout] = mmav_configure_case(U);
polar_info = mmav_print_vlm_polar_info(vlm_cfg);
gap = mmav_tail_wing_gap_info(cfg);

active_cg_B_mm = U.active_cg_B_mm;

if isfield(U,'controlled_trim') && isfield(U.controlled_trim,'velocity_sweep_V_mps') && ~isempty(U.controlled_trim.velocity_sweep_V_mps)
    V_vec = reshape(U.controlled_trim.velocity_sweep_V_mps, [], 1);
elseif isfield(U.eom,'velocity_sweep_V_mps') && ~isempty(U.eom.velocity_sweep_V_mps)
    V_vec = reshape(U.eom.velocity_sweep_V_mps, [], 1);
else
    V_vec = (5.0:0.25:10.0).';
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
fprintf('Velocity scan      : %.2f to %.2f m/s, N=%d\n', min(V_vec), max(V_vec), numel(V_vec));
fprintf('Unknown x          : [alpha gamma wing_collective tail_curl_delta]\n');
fprintf('Initial guess x0   : [%.2f %.2f %.2f %.2f]\n', x0);

Rows = table();
TrimResults = cell(numel(V_vec),1);

for ii = 1:numel(V_vec)
    V = V_vec(ii);
    fprintf('  V = %.2f m/s...\n', V);

    try
        trim = mmav_solve_controlled_trim_at_speed(cfg, vlm_cfg, active_cg_B_mm, V, x0, bounds, options);
        TrimResults{ii} = trim;

        if isfinite(trim.residual_norm) && trim.residual_norm < 0.05
            x0 = trim.x;   % continuation: use previous trim as next initial guess
        end
    catch ME
        warning('Controlled trim solve failed at V=%.2f: %s', V, ME.message);
        trim = empty_controlled_trim_result(V);
        TrimResults{ii} = trim;
    end

    pass_speed_point = trim.trim_residual_ok && abs(trim.detail.Cm_y) <= U.screen_eom.max_abs_Cm && ...
        trim.dCm_dalpha_per_rad <= U.screen_eom.max_dCm_dalpha && ...
        trim.detail.LD >= U.screen_eom.min_LD && ...
        gap.overlap_each_side_mm <= U.screen_eom.max_tail_overlap_mm && trim.vlm_converged;

    row = table(V, trim.alpha_deg, trim.gamma_deg, trim.theta_deg, ...
        trim.wing_collective_deg, trim.wing_R_incidence_deg, trim.wing_L_incidence_deg, ...
        trim.tail_curl_delta_deg, trim.tail_controlled_curl_deg, ...
        trim.residual(1), trim.residual(2), trim.residual(3), trim.residual_norm, ...
        trim.detail.Cm_y, trim.dCm_dalpha_per_rad, ...
        trim.detail.CL_wind, trim.detail.CD_wind, trim.detail.LD, ...
        trim.detail.L_over_W, trim.detail.D_over_W, ...
        trim.vlm_converged, trim.detail.solver_err, trim.exitflag, trim.trim_residual_ok, pass_speed_point, ...
        'VariableNames', {'V_mps','alpha_deg','gamma_deg','theta_deg', ...
        'wing_collective_deg','wing_R_incidence_deg','wing_L_incidence_deg', ...
        'tail_curl_delta_deg','tail_curl_total_deg', ...
        'R_lift','R_drag','R_Cm','residual_norm', ...
        'Cm_y','dCm_dalpha','CL','CD','LD','L_over_W','D_over_W', ...
        'vlm_converged','vlm_solver_err','exitflag','trim_residual_ok','pass_speed_point'});
    Rows = [Rows; row]; %#ok<AGROW>

    fprintf(['    [CTRL-SWEEP] V=%.2f alpha=%.2f gamma=%.2f wingC=%+.2f tailCurlDelta=%+.2f | ', ...
        'R=[%+.2e %+.2e %+.2e] norm=%.2e | LD=%.2f pass=%d\n'], ...
        V, trim.alpha_deg, trim.gamma_deg, trim.wing_collective_deg, trim.tail_curl_delta_deg, ...
        trim.residual(1), trim.residual(2), trim.residual(3), trim.residual_norm, trim.detail.LD, pass_speed_point);
end

fprintf('\n--- Controlled velocity-sweep trim table ---\n');
disp(Rows);

fprintf('\nInterpretation:\n');
fprintf('  This scan fixes V and solves alpha/gamma plus controls.\n');
fprintf('  Controls are symmetric wing incidence and tail curl deformation.\n');
fprintf('  pass_speed_point requires small trim residual, negative dCm/dalpha, L/D screen, no tail overlap, and VLM convergence.\n');

%% Auto-save result and settings
results_dir = fullfile(project_dir, 'results', 'controlled_velocity_sweep');
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
settings.control_bounds = bounds;
settings.solver_options = options;
settings.polar_info = polar_info;
settings.control_unknowns = {'alpha_deg','gamma_deg','wing_collective_deg','tail_curl_delta_deg'};

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
result_filename = sprintf('RUN02E_CTRL_%s_%s_%s_%s.mat', V_tag, CG_tag, Tail_tag, Time_tag);
result_filename = local_clean_filename(result_filename);
result_path = fullfile(results_dir, result_filename);

save(result_path, 'Rows', 'TrimResults', 'settings', 'U', 'cfg', 'layout', 'gap');
csv_path = strrep(result_path, '.mat', '.csv');
writetable(Rows, csv_path);

fprintf('\n[SAVE-DIAG] Saved MAT: %s\n', result_path);
fprintf('[SAVE-DIAG] Saved CSV: %s\n', csv_path);

%% Simple plots
figure('Name','RUN 02E: controlled velocity sweep states','Color','w');
subplot(3,1,1); plot(Rows.V_mps, Rows.alpha_deg, 'o-', 'LineWidth',1.2); ylabel('\alpha [deg]'); grid on;
subplot(3,1,2); plot(Rows.V_mps, Rows.gamma_deg, 'o-', 'LineWidth',1.2); ylabel('\gamma [deg]'); grid on;
subplot(3,1,3); plot(Rows.V_mps, Rows.theta_deg, 'o-', 'LineWidth',1.2); ylabel('\theta [deg]'); xlabel('V [m/s]'); grid on;

figure('Name','RUN 02E: controlled velocity sweep controls','Color','w');
subplot(2,1,1); plot(Rows.V_mps, Rows.wing_collective_deg, 'o-', 'LineWidth',1.2); ylabel('wing collective [deg]'); grid on;
subplot(2,1,2); plot(Rows.V_mps, Rows.tail_curl_delta_deg, 'o-', 'LineWidth',1.2); ylabel('tail curl \Delta [deg]'); xlabel('V [m/s]'); grid on;

figure('Name','RUN 02E: controlled velocity sweep residuals','Color','w');
subplot(3,1,1); plot(Rows.V_mps, Rows.R_lift, 'o-', 'LineWidth',1.2); ylabel('R_{lift}'); grid on;
subplot(3,1,2); plot(Rows.V_mps, Rows.R_drag, 'o-', 'LineWidth',1.2); ylabel('R_{drag}'); grid on;
subplot(3,1,3); plot(Rows.V_mps, Rows.R_Cm, 'o-', 'LineWidth',1.2); ylabel('R_{Cm}'); xlabel('V [m/s]'); grid on;

figure('Name','RUN 02E: controlled velocity sweep aero','Color','w');
subplot(3,1,1); plot(Rows.V_mps, Rows.CL, 'o-', 'LineWidth',1.2); ylabel('C_L'); grid on;
subplot(3,1,2); plot(Rows.V_mps, Rows.CD, 'o-', 'LineWidth',1.2); ylabel('C_D'); grid on;
subplot(3,1,3); plot(Rows.V_mps, Rows.LD, 'o-', 'LineWidth',1.2); ylabel('L/D'); xlabel('V [m/s]'); grid on;

% =====================================================================
function trim = empty_controlled_trim_result(V)
trim = struct();
trim.V_mps = V;
trim.alpha_deg = NaN;
trim.gamma_deg = NaN;
trim.theta_deg = NaN;
trim.wing_collective_deg = NaN;
trim.wing_R_incidence_deg = NaN;
trim.wing_L_incidence_deg = NaN;
trim.tail_curl_delta_deg = NaN;
trim.tail_controlled_curl_deg = NaN;
trim.residual = [Inf; Inf; Inf];
trim.residual_norm = Inf;
trim.dCm_dalpha_per_rad = NaN;
trim.exitflag = NaN;
trim.trim_residual_ok = false;
trim.vlm_converged = false;
trim.x = [NaN NaN NaN NaN];
trim.detail = struct('Cm_y',NaN,'CL_wind',NaN,'CD_wind',NaN,'LD',NaN, ...
    'L_over_W',NaN,'D_over_W',NaN,'solver_err',NaN);
end

% =====================================================================
function v = local_getfield(s, field, default)
if isstruct(s) && isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end

% =====================================================================
function tag = local_numtag(x, fmt)
if ~isfinite(x)
    tag = 'NaN';
    return;
end
s = sprintf(fmt, x);
s = strrep(s, '-', 'm');
s = strrep(s, '+', 'p');
s = strrep(s, '.', 'p');
tag = s;
end

% =====================================================================
function tag = local_clean_tag(s)
tag = char(string(s));
tag = regexprep(tag, '[^A-Za-z0-9]+', '_');
tag = regexprep(tag, '_+', '_');
tag = regexprep(tag, '^_|_$', '');
end

% =====================================================================
function name = local_clean_filename(name)
name = regexprep(name, '[<>:"/\\|?*]', '_');
name = regexprep(name, '_+', '_');
end
