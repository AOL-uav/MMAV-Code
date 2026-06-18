function trim = mmav_solve_steady_glide_trim(cfg, vlm_cfg, user_cg_B_mm, z0, bounds, options)
%MMAV_SOLVE_STEADY_GLIDE_TRIM Solve nonlinear algebraic steady-glide trim.
%
% trim = mmav_solve_steady_glide_trim(cfg, vlm_cfg, user_cg_B_mm, z0, bounds, options)
%
% Solves for z = [V_mps, alpha_deg, gamma_deg] such that
%   L - W*cos(gamma) = 0
%   D - W*sin(gamma) = 0
%   Cm_y             = 0
%
% gamma is positive downward.  This routine intentionally leaves the old
% fixed-speed alpha sweep untouched; this is the EOM-style trim solve where
% alpha is an unknown state-derived quantity.
%
% Solver implementation:
%   Uses fminsearch on a bounded sigmoid transform, so it does not require
%   Optimization Toolbox.  If the Optimization Toolbox is available, users can
%   later replace this wrapper with fsolve/lsqnonlin without changing the
%   residual function.

if nargin < 6 || isempty(options), options = struct(); end
if nargin < 5 || isempty(bounds)
    bounds.lb = [3.5, -4.0, 0.0];
    bounds.ub = [8.0, 14.0, 35.0];
end
if nargin < 4 || isempty(z0), z0 = [5.8, 5.0, 13.0]; end
if nargin < 3, user_cg_B_mm = []; end
if nargin < 2 || isempty(vlm_cfg), vlm_cfg = mmav_default_vlm_config(); end

rho = getfield_default(options, 'rho_kgpm3', 1.225);
weights = getfield_default(options, 'residual_weights', [1.0; 1.0; 5.0]);
weights = reshape(weights, 3, 1);
max_iter = getfield_default(options, 'max_iter', 120);
max_fun_evals = getfield_default(options, 'max_fun_evals', 260);
tol_x = getfield_default(options, 'tol_x', 1e-3);
tol_fun = getfield_default(options, 'tol_fun', 1e-6);
stab_dalpha_deg = getfield_default(options, 'stability_dalpha_deg', 0.25);

lb = reshape(bounds.lb, 1, 3);
ub = reshape(bounds.ub, 1, 3);
z0 = min(max(reshape(z0,1,3), lb + 1e-6), ub - 1e-6);
y0 = to_unconstrained(z0, lb, ub);

history_y = [];
history_J = [];

    function stop = outfun(y, optimValues, state) %#ok<INUSL>
        stop = false;
        if strcmp(state,'iter') || strcmp(state,'init') || strcmp(state,'done')
            history_y(end+1,:) = reshape(y,1,[]); %#ok<AGROW>
            history_J(end+1,1) = optimValues.fval; %#ok<AGROW>
        end
    end

    function J = objective_y(y)
        z = from_unconstrained(y, lb, ub);
        [R,detail] = mmav_steady_glide_trim_residual(z, cfg, vlm_cfg, user_cg_B_mm, rho);
        if any(~isfinite(R)) || ~isfield(detail,'converged') || ~detail.converged
            J = 1e6 + nansum_safe((weights .* finite_or_large(R)).^2);
            return;
        end
        J = sum((weights .* R).^2);
        if ~isfinite(J)
            J = 1e9;
        end
    end

opts = optimset('Display','off', ...
                'MaxIter',max_iter, ...
                'MaxFunEvals',max_fun_evals, ...
                'TolX',tol_x, ...
                'TolFun',tol_fun, ...
                'OutputFcn',@outfun);

[y_opt, fval, exitflag, output] = fminsearch(@objective_y, y0, opts);
z_opt = from_unconstrained(y_opt, lb, ub);
[R_opt, detail] = mmav_steady_glide_trim_residual(z_opt, cfg, vlm_cfg, user_cg_B_mm, rho);

% Static-stability slope at the trim solution.  This is the local
% aerodynamic dCm/dalpha at fixed V and fixed geometry.
dCm_dalpha_per_rad = NaN;
try
    ap = detail.alpha_deg + stab_dalpha_deg;
    am = detail.alpha_deg - stab_dalpha_deg;
    [~,dp] = mmav_steady_glide_trim_residual([detail.V_mps, ap, detail.gamma_deg], cfg, vlm_cfg, user_cg_B_mm, rho);
    [~,dm] = mmav_steady_glide_trim_residual([detail.V_mps, am, detail.gamma_deg], cfg, vlm_cfg, user_cg_B_mm, rho);
    dCm_dalpha_per_deg = (dp.Cm_y - dm.Cm_y) / (2*stab_dalpha_deg);
    dCm_dalpha_per_rad = dCm_dalpha_per_deg * (180/pi);
catch ME
    warning('Static-stability finite difference failed: %s', ME.message);
end

trim = struct();
trim.model = 'steady-glide nonlinear algebraic EOM trim';
trim.z = z_opt;
trim.V_mps = detail.V_mps;
trim.alpha_deg = detail.alpha_deg;
trim.gamma_deg = detail.gamma_deg;
trim.theta_deg = detail.theta_deg;
trim.u_forward_mps = detail.u_forward_mps;
trim.v_right_mps = detail.v_right_mps;
trim.w_down_mps = detail.w_down_mps;
trim.p_radps = 0.0;
trim.q_radps = 0.0;
trim.r_radps = 0.0;
trim.phi_deg = 0.0;
trim.beta_deg = 0.0;
trim.residual = R_opt;
trim.residual_norm = norm(R_opt);
trim.objective = fval;
trim.exitflag = exitflag;
trim.output = output;
trim.detail = detail;
trim.dCm_dalpha_per_rad = dCm_dalpha_per_rad;
trim.static_stable = isfinite(dCm_dalpha_per_rad) && dCm_dalpha_per_rad < 0;
trim.bounds = bounds;
trim.options = options;
trim.history_y = history_y;
trim.history_objective = history_J;

trim.eom_residual_ok = isfinite(trim.residual_norm) && trim.residual_norm <= getfield_default(options,'trim_residual_tol',0.02);
trim.vlm_converged = isfield(detail,'converged') && detail.converged;
trim.summary_table = make_summary_table(trim);
end

% =====================================================================
function T = make_summary_table(trim)
V_mps = trim.V_mps;
alpha_deg = trim.alpha_deg;
gamma_deg = trim.gamma_deg;
theta_deg = trim.theta_deg;
u_forward_mps = trim.u_forward_mps;
w_down_mps = trim.w_down_mps;
R_L = trim.residual(1);
R_D = trim.residual(2);
R_Cm = trim.residual(3);
residual_norm = trim.residual_norm;
Cm_y = trim.detail.Cm_y;
dCm_dalpha = trim.dCm_dalpha_per_rad;
CL = trim.detail.CL_wind;
CD = trim.detail.CD_wind;
LD = trim.detail.LD;
L_over_W = trim.detail.L_over_W;
D_over_W = trim.detail.D_over_W;
vlm_converged = trim.detail.converged;
eom_residual_ok = trim.eom_residual_ok;
optimizer_exitflag = trim.exitflag;
solver_err = trim.detail.solver_err;
T = table(V_mps, alpha_deg, gamma_deg, theta_deg, u_forward_mps, w_down_mps, ...
          R_L, R_D, R_Cm, residual_norm, Cm_y, dCm_dalpha, CL, CD, LD, ...
          L_over_W, D_over_W, vlm_converged, eom_residual_ok, solver_err, optimizer_exitflag);
end

% =====================================================================
function y = to_unconstrained(x, lb, ub)
s = (x - lb) ./ max(ub - lb, 1e-12);
s = min(max(s, 1e-8), 1-1e-8);
y = log(s ./ (1-s));
end

% =====================================================================
function x = from_unconstrained(y, lb, ub)
y = reshape(y, 1, 3);
s = 1 ./ (1 + exp(-y));
x = lb + (ub - lb) .* s;
end

% =====================================================================
function v = getfield_default(s, field, default)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end

% =====================================================================
function y = finite_or_large(x)
y = x;
y(~isfinite(y)) = 1e3;
end

% =====================================================================
function s = nansum_safe(x)
x = x(isfinite(x));
if isempty(x)
    s = 1e9;
else
    s = sum(x);
end
end
