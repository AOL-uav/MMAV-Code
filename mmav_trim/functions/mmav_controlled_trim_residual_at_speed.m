function [R, detail] = mmav_controlled_trim_residual_at_speed(x, cfg, vlm_cfg, user_cg_B_mm, V_mps, rho_kgpm3)
%MMAV_CONTROLLED_TRIM_RESIDUAL_AT_SPEED Conventional fixed-speed trim residual.
%
% Unknown vector:
%   x = [alpha_deg, gamma_deg, wing_collective_deg, tail_curl_delta_deg]
%
% Controls:
%   wing_collective_deg  : common incidence command to both wings
%   tail_curl_delta_deg  : deformation command added to baseline tail curl
%
% Differential wing incidence is held at zero here because this is a
% symmetric longitudinal trim solve.  The aero-state layer already supports
% right/left incidence and differential incidence for a later 6-DOF trim.
%
% Residual:
%   R(1) = L/W - cos(gamma)
%   R(2) = D/W - sin(gamma)
%   R(3) = Cm_y
%
% Optional diagnostic line:
%   [CTRL-RESID]
%
% Enable with any of these flags:
%   vlm_cfg.diagnostics = true;
%   vlm_cfg.diagnostics_controlled_residual = true;
%   vlm_cfg.diagnostics_residual = true;
%   cfg.diagnostics = true;
%   state.diagnostics = true;

if nargin < 6 || isempty(rho_kgpm3), rho_kgpm3 = 1.225; end
if nargin < 5 || isempty(V_mps), V_mps = 5.8; end
if nargin < 4, user_cg_B_mm = []; end
if nargin < 3 || isempty(vlm_cfg), vlm_cfg = mmav_default_vlm_config(); end

x = reshape(x, 1, []);
if numel(x) ~= 4 || any(~isfinite(x))
    R = [Inf; Inf; Inf];
    detail = empty_detail(V_mps, x);
    return;
end

alpha_deg = x(1);
gamma_deg = x(2);
wing_collective_deg = x(3);
tail_curl_delta_deg = x(4);

state = struct();
state.rho_kgpm3 = rho_kgpm3;
state.V_mps = V_mps;
state.alpha_deg = alpha_deg;
state.beta_deg = 0.0;
state.pitch_deg = 0.0;
state.control = struct();
state.control.wing_collective_deg = wing_collective_deg;
state.control.wing_differential_deg = 0.0;
state.control.tail_curl_delta_deg = tail_curl_delta_deg;
state.control.tail_incidence_delta_deg = 0.0;

try
    [out,~] = mmav_eval_vlm_aero_state(cfg, user_cg_B_mm, state, vlm_cfg, []);
catch ME
    warning('Controlled trim residual aero evaluation failed: %s', ME.message);
    R = [Inf; Inf; Inf];
    detail = empty_detail(V_mps, x);
    detail.error_message = ME.message;

    if mmav_ctrl_diag_enabled(cfg, vlm_cfg, state)
        theta_deg = alpha_deg - gamma_deg;
        base_curl_deg = local_getfield(cfg.curled_tail, 'curl_deg', NaN);
        tail_total_deg = base_curl_deg + tail_curl_delta_deg;
        fprintf(['[CTRL-RESID] ', ...
            'V=%.3f alpha=%.3f gamma=%.3f theta=%.3f | ', ...
            'wing=%.3f tail_d=%.3f tail=%.3f | AERO_EVAL_FAILED | %s\n'], ...
            V_mps, alpha_deg, gamma_deg, theta_deg, ...
            wing_collective_deg, tail_curl_delta_deg, tail_total_deg, ME.message);
    end
    return;
end

W_N = out.vehicle.mass.total_mass_kg * 9.80665;
qS  = max(out.q_Pa * out.Sref_m2, 1e-12);
L_N = out.CL_wind * qS;
D_N = out.CD_wind * qS;

R = [ (L_N - W_N*cosd(gamma_deg)) / W_N; ...
      (D_N - W_N*sind(gamma_deg)) / W_N; ...
      out.Cm_y ];

u_forward_mps = V_mps*cosd(alpha_deg);
w_down_mps    = V_mps*sind(alpha_deg);
theta_deg     = alpha_deg - gamma_deg;
LD = L_N / max(D_N, 1e-12);
gamma_LD_deg = rad2deg(atan2(max(D_N,0), max(abs(L_N),1e-12)));

if isfield(out.vlm_result_lifting,'converged')
    converged = out.vlm_result_lifting.converged;
else
    converged = true;
end
if isfield(out.vlm_result_lifting,'err')
    solver_err = out.vlm_result_lifting.err;
else
    solver_err = NaN;
end
if isfield(out.vlm_result_lifting,'iter')
    solver_iter = out.vlm_result_lifting.iter;
else
    solver_iter = NaN;
end

detail = struct();
detail.V_mps = V_mps;
detail.alpha_deg = alpha_deg;
detail.gamma_deg = gamma_deg;
detail.theta_deg = theta_deg;
detail.u_forward_mps = u_forward_mps;
detail.w_down_mps = w_down_mps;
detail.wing_collective_deg = wing_collective_deg;
detail.wing_differential_deg = 0.0;
detail.wing_R_incidence_deg = wing_collective_deg;
detail.wing_L_incidence_deg = wing_collective_deg;
detail.tail_curl_delta_deg = tail_curl_delta_deg;
detail.tail_controlled_curl_deg = cfg.curled_tail.curl_deg + tail_curl_delta_deg;
detail.L_N = L_N;
detail.D_N = D_N;
detail.W_N = W_N;
detail.L_over_W = L_N/W_N;
detail.D_over_W = D_N/W_N;
detail.LD = LD;
detail.glide_gamma_from_LD_deg = gamma_LD_deg;
detail.Cm_y = out.Cm_y;
detail.CL_wind = out.CL_wind;
detail.CD_wind = out.CD_wind;
detail.CF_B = out.CF_B;
detail.CM_CG_B = out.CM_CG_B;
detail.F_B_N = out.loads.F_B_N;
detail.M_CG_B_Nm = out.loads.M_CG_B_Nm;
detail.q_Pa = out.q_Pa;
detail.Sref_m2 = out.Sref_m2;
detail.cref_m = out.cref_m;
detail.converged = converged;
detail.solver_err = solver_err;
detail.solver_iter = solver_iter;
detail.out = out;
detail.source_table = out.loads.tables.by_source;
detail.error_message = '';

% Simple recognizable controlled-trim residual diagnostic line.
% Enable with either:
%   vlm_cfg.diagnostics = true;
%   vlm_cfg.diagnostics_controlled_residual = true;
%   vlm_cfg.diagnostics_residual = true;
%   state.diagnostics = true;
%   cfg.diagnostics = true;
if mmav_ctrl_diag_enabled(cfg, vlm_cfg, state)
    fprintf(['[CTRL-RESID] ', ...
        'V=%.3f alpha=%.3f gamma=%.3f theta=%.3f | ', ...
        'wing=%.3f tail_d=%.3f tail=%.3f | ', ...
        'R=[%+.3e %+.3e %+.3e] | ', ...
        'L/W=%.4f D/W=%.4f Cm=%+.4f | ', ...
        'CL=%.4f CD=%.4f L/D=%.3f gamma_LD=%.3f | ', ...
        'VLM=%d err=%.2e iter=%g\n'], ...
        detail.V_mps, detail.alpha_deg, detail.gamma_deg, detail.theta_deg, ...
        detail.wing_collective_deg, detail.tail_curl_delta_deg, detail.tail_controlled_curl_deg, ...
        R(1), R(2), R(3), ...
        detail.L_over_W, detail.D_over_W, detail.Cm_y, ...
        detail.CL_wind, detail.CD_wind, detail.LD, detail.glide_gamma_from_LD_deg, ...
        detail.converged, detail.solver_err, detail.solver_iter);
end

end

% =====================================================================
function tf = mmav_ctrl_diag_enabled(cfg, vlm_cfg, state)
%MMAV_CTRL_DIAG_ENABLED Local flag reader for controlled residual diagnostics.
% Checks both generic flags and the controlled-residual-specific flag.
tf = false;
tf = tf || mmav_diag_enabled(cfg, vlm_cfg, state, 'diagnostics_controlled_residual');
tf = tf || mmav_diag_enabled(cfg, vlm_cfg, state, 'diagnostics_residual');
end

% =====================================================================
function tf = mmav_diag_enabled(cfg, vlm_cfg, state, specific_field)
%MMAV_DIAG_ENABLED Small local flag reader for optional console diagnostics.
% Checks both generic and function-specific flags, without requiring any
% particular config schema.
tf = false;
tf = tf || mmav_local_flag(vlm_cfg, 'diagnostics');
tf = tf || mmav_local_flag(vlm_cfg, specific_field);
tf = tf || mmav_local_flag(state, 'diagnostics');
tf = tf || mmav_local_flag(state, specific_field);
tf = tf || mmav_local_flag(cfg, 'diagnostics');
tf = tf || mmav_local_flag(cfg, specific_field);
if isstruct(vlm_cfg) && isfield(vlm_cfg,'diag')
    tf = tf || mmav_local_flag(vlm_cfg.diag, 'enabled');
    tf = tf || mmav_local_flag(vlm_cfg.diag, specific_field);
end
if isstruct(cfg) && isfield(cfg,'diag')
    tf = tf || mmav_local_flag(cfg.diag, 'enabled');
    tf = tf || mmav_local_flag(cfg.diag, specific_field);
end
end

% =====================================================================
function tf = mmav_local_flag(s, field)
tf = false;
if ~isstruct(s) || ~isfield(s, field) || isempty(s.(field))
    return;
end
v = s.(field);
if islogical(v) || isnumeric(v)
    tf = any(v(:) ~= 0);
elseif ischar(v) || isstring(v)
    try
        txt = lower(strtrim(char(string(v))));
        tf = any(strcmp(txt, {'true','on','yes','1','enable','enabled'}));
    catch
        tf = false;
    end
elseif isstruct(v)
    tf = mmav_local_flag(v, 'enabled');
end
end

% =====================================================================
function v = local_getfield(s, field, default_value)
if isstruct(s) && isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end

% =====================================================================
function detail = empty_detail(V_mps, x)
if numel(x) >= 4
    a = x(1); g = x(2); dw = x(3); dt = x(4);
else
    a = NaN; g = NaN; dw = NaN; dt = NaN;
end
detail = struct('V_mps',V_mps,'alpha_deg',a,'gamma_deg',g,'theta_deg',NaN, ...
    'u_forward_mps',NaN,'w_down_mps',NaN, ...
    'wing_collective_deg',dw,'wing_differential_deg',0, ...
    'wing_R_incidence_deg',dw,'wing_L_incidence_deg',dw, ...
    'tail_curl_delta_deg',dt,'tail_controlled_curl_deg',NaN, ...
    'L_N',NaN,'D_N',NaN,'W_N',NaN,'L_over_W',NaN,'D_over_W',NaN, ...
    'LD',NaN,'glide_gamma_from_LD_deg',NaN,'Cm_y',NaN, ...
    'CL_wind',NaN,'CD_wind',NaN,'CF_B',[NaN NaN NaN],'CM_CG_B',[NaN NaN NaN], ...
    'F_B_N',[NaN NaN NaN],'M_CG_B_Nm',[NaN NaN NaN], ...
    'q_Pa',NaN,'Sref_m2',NaN,'cref_m',NaN, ...
    'converged',false,'solver_err',NaN,'solver_iter',NaN, ...
    'out',[],'source_table',table(),'error_message','');
end
