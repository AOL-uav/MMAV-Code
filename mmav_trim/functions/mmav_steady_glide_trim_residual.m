function [R, detail] = mmav_steady_glide_trim_residual(z, cfg, vlm_cfg, user_cg_B_mm, rho_kgpm3)
%MMAV_STEADY_GLIDE_TRIM_RESIDUAL Residual for unpowered symmetric steady-glide trim.
%
% Unknown vector:
%   z = [V_mps, alpha_deg, gamma_deg]
%
% where gamma_deg is positive downward.  The reduced steady-glide equations are
%
%   L(V,alpha) - W*cos(gamma) = 0
%   D(V,alpha) - W*sin(gamma) = 0
%   Cm_y(V,alpha)             = 0
%
% This is the algebraic trim form corresponding to straight, symmetric,
% unpowered glide with p=q=r=0, beta=0, phi=0.  The angle of attack is solved
% as part of z rather than imposed by an alpha sweep.
%
% Outputs:
%   R(1) = normal-force / lift balance residual, normalized by W
%   R(2) = path-axis drag balance residual, normalized by W
%   R(3) = pitch moment coefficient about active CG
%
% Body/build frame B used by the model:
%   x_B aft, y_B right, z_B up.
%
% Conventional longitudinal state equivalent:
%   u_forward = V*cos(alpha), w_down = V*sin(alpha), theta = alpha - gamma.

if nargin < 5 || isempty(rho_kgpm3), rho_kgpm3 = 1.225; end
if nargin < 4, user_cg_B_mm = []; end
if nargin < 3 || isempty(vlm_cfg), vlm_cfg = mmav_default_vlm_config(); end

z = reshape(z, 1, []);
if numel(z) ~= 3 || any(~isfinite(z))
    R = [Inf; Inf; Inf];
    detail = empty_detail(z);
    return;
end

V_mps     = z(1);
alpha_deg = z(2);
gamma_deg = z(3);

state = struct('rho_kgpm3',rho_kgpm3, ...
               'V_mps',V_mps, ...
               'alpha_deg',alpha_deg, ...
               'beta_deg',0.0, ...
               'pitch_deg',0.0);

try
    [out,~] = mmav_eval_vlm_aero_state(cfg, user_cg_B_mm, state, vlm_cfg, []);
catch ME
    warning('EOM trim residual VLM evaluation failed: %s', ME.message);
    R = [Inf; Inf; Inf];
    detail = empty_detail(z);
    detail.error_message = ME.message;
    if mmav_diag_enabled(cfg, vlm_cfg, state, 'diagnostics_residual')
        fprintf(['[EOM-RESID] ', ...
            'V=%.3f alpha=%.3f gamma=%.3f | VLM_EVAL_FAILED | %s\n'], ...
            V_mps, alpha_deg, gamma_deg, ME.message);
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

% Conventional longitudinal state representation, using x-forward, z-down
% aircraft axes for readability.  The internal B-frame velocity is the
% opposite of the drag direction: [-u_forward, 0, -w_down].
u_forward_mps = V_mps*cosd(alpha_deg);
w_down_mps    = V_mps*sind(alpha_deg);
theta_deg     = alpha_deg - gamma_deg;

lift_drag_gamma_deg = rad2deg(atan2(max(D_N,0), max(abs(L_N),1e-12)));
LD = L_N / max(D_N, 1e-12);

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
detail.v_right_mps = 0.0;
detail.w_down_mps = w_down_mps;
detail.p_radps = 0.0;
detail.q_radps = 0.0;
detail.r_radps = 0.0;
detail.phi_deg = 0.0;
detail.beta_deg = 0.0;
detail.L_N = L_N;
detail.D_N = D_N;
detail.W_N = W_N;
detail.L_over_W = L_N/W_N;
detail.D_over_W = D_N/W_N;
detail.LD = LD;
detail.glide_gamma_from_LD_deg = lift_drag_gamma_deg;
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

% Simple recognizable residual diagnostic line.  Enable with either:
%   vlm_cfg.diagnostics = true;
%   vlm_cfg.diagnostics_residual = true;
%   state.diagnostics = true;
%   cfg.diagnostics = true;
if mmav_diag_enabled(cfg, vlm_cfg, state, 'diagnostics_residual')
    fprintf(['[EOM-RESID] ', ...
        'V=%.3f alpha=%.3f gamma=%.3f theta=%.3f | ', ...
        'R=[%+.3e %+.3e %+.3e] | ', ...
        'L/W=%.4f D/W=%.4f Cm=%+.4f | ', ...
        'CL=%.4f CD=%.4f L/D=%.3f gamma_LD=%.3f | ', ...
        'VLM=%d err=%.2e iter=%g\n'], ...
        detail.V_mps, detail.alpha_deg, detail.gamma_deg, detail.theta_deg, ...
        R(1), R(2), R(3), ...
        detail.L_over_W, detail.D_over_W, detail.Cm_y, ...
        detail.CL_wind, detail.CD_wind, detail.LD, detail.glide_gamma_from_LD_deg, ...
        detail.converged, detail.solver_err, detail.solver_iter);
end

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
function detail = empty_detail(z)
if numel(z) >= 3
    V = z(1); a = z(2); g = z(3);
else
    V = NaN; a = NaN; g = NaN;
end
detail = struct('V_mps',V,'alpha_deg',a,'gamma_deg',g,'theta_deg',NaN, ...
    'u_forward_mps',NaN,'v_right_mps',NaN,'w_down_mps',NaN, ...
    'p_radps',0,'q_radps',0,'r_radps',0,'phi_deg',0,'beta_deg',0, ...
    'L_N',NaN,'D_N',NaN,'W_N',NaN,'L_over_W',NaN,'D_over_W',NaN, ...
    'LD',NaN,'glide_gamma_from_LD_deg',NaN,'Cm_y',NaN, ...
    'CL_wind',NaN,'CD_wind',NaN,'CF_B',[NaN NaN NaN],'CM_CG_B',[NaN NaN NaN], ...
    'F_B_N',[NaN NaN NaN],'M_CG_B_Nm',[NaN NaN NaN], ...
    'q_Pa',NaN,'Sref_m2',NaN,'cref_m',NaN, ...
    'converged',false,'solver_err',NaN,'solver_iter',NaN, ...
    'out',[],'source_table',table(),'error_message','');
end
