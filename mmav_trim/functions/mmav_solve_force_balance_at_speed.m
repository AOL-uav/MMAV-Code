function fb = mmav_solve_force_balance_at_speed(cfg, vlm_cfg, user_cg_B_mm, V_mps, x0, bounds, options)
%MMAV_SOLVE_FORCE_BALANCE_AT_SPEED Solve glide force balance at fixed airspeed.
%
% fb = mmav_solve_force_balance_at_speed(cfg, vlm_cfg, user_cg_B_mm, V_mps, x0, bounds, options)
%
% This is a diagnostic continuation tool, not a full passive trim solve.
% It fixes V and solves x = [alpha_deg, gamma_deg] for
%   L(V,alpha) - W*cos(gamma) = 0
%   D(V,alpha) - W*sin(gamma) = 0
% Then it reports the remaining pitch-moment residual Cm_y.
%
% For a passive vehicle with no elevator/thrust/other trim control, an
% arbitrary V generally cannot satisfy all three equations.  The free EOM
% trim solver should be used to solve [V, alpha, gamma] simultaneously.
% This routine is useful for a velocity sweep showing where Cm_y crosses zero.

if nargin < 7 || isempty(options), options = struct(); end
if nargin < 6 || isempty(bounds)
    bounds.lb = [-4.0, 0.0];
    bounds.ub = [14.0, 35.0];
end
if nargin < 5 || isempty(x0), x0 = [5.0, 13.0]; end
if nargin < 4 || isempty(V_mps), V_mps = 5.8; end
if nargin < 3, user_cg_B_mm = []; end
if nargin < 2 || isempty(vlm_cfg), vlm_cfg = mmav_default_vlm_config(); end

rho = getfield_default_local(options, 'rho_kgpm3', 1.225);
weights = getfield_default_local(options, 'force_weights', [1.0; 1.0]);
weights = reshape(weights, 2, 1);
max_iter = getfield_default_local(options, 'max_iter', 80);
max_fun_evals = getfield_default_local(options, 'max_fun_evals', 180);
tol_x = getfield_default_local(options, 'tol_x', 1e-3);
tol_fun = getfield_default_local(options, 'tol_fun', 1e-8);
stab_dalpha_deg = getfield_default_local(options, 'stability_dalpha_deg', 0.25);

lb = reshape(bounds.lb, 1, 2);
ub = reshape(bounds.ub, 1, 2);
x0 = min(max(reshape(x0,1,2), lb + 1e-6), ub - 1e-6);
y0 = to_unconstrained_local(x0, lb, ub);

    function J = objective_y(y)
        x = from_unconstrained_local(y, lb, ub);
        z = [V_mps, x(1), x(2)];
        [R,detail] = mmav_steady_glide_trim_residual(z, cfg, vlm_cfg, user_cg_B_mm, rho);
        if any(~isfinite(R(1:2))) || ~isfield(detail,'converged') || ~detail.converged
            J = 1e6 + sum((weights .* finite_or_large_local(R(1:2))).^2);
            return;
        end
        J = sum((weights .* R(1:2)).^2);
        if ~isfinite(J), J = 1e9; end
    end

opts = optimset('Display','off', ...
                'MaxIter',max_iter, ...
                'MaxFunEvals',max_fun_evals, ...
                'TolX',tol_x, ...
                'TolFun',tol_fun);

[y_opt, fval, exitflag, output] = fminsearch(@objective_y, y0, opts);
x_opt = from_unconstrained_local(y_opt, lb, ub);
z_opt = [V_mps, x_opt(1), x_opt(2)];
[R_opt, detail] = mmav_steady_glide_trim_residual(z_opt, cfg, vlm_cfg, user_cg_B_mm, rho);

% Static-stability slope at fixed V and force-balance gamma.
dCm_dalpha_per_rad = NaN;
try
    ap = detail.alpha_deg + stab_dalpha_deg;
    am = detail.alpha_deg - stab_dalpha_deg;
    [~,dp] = mmav_steady_glide_trim_residual([V_mps, ap, detail.gamma_deg], cfg, vlm_cfg, user_cg_B_mm, rho);
    [~,dm] = mmav_steady_glide_trim_residual([V_mps, am, detail.gamma_deg], cfg, vlm_cfg, user_cg_B_mm, rho);
    dCm_dalpha_per_deg = (dp.Cm_y - dm.Cm_y) / (2*stab_dalpha_deg);
    dCm_dalpha_per_rad = dCm_dalpha_per_deg * (180/pi);
catch ME
    warning('Fixed-speed stability finite difference failed: %s', ME.message);
end

fb = struct();
fb.model = 'fixed-speed force-balance diagnostic';
fb.V_mps = V_mps;
fb.alpha_deg = detail.alpha_deg;
fb.gamma_deg = detail.gamma_deg;
fb.theta_deg = detail.theta_deg;
fb.u_forward_mps = detail.u_forward_mps;
fb.w_down_mps = detail.w_down_mps;
fb.residual = R_opt;
fb.force_residual = R_opt(1:2);
fb.force_residual_norm = norm(R_opt(1:2));
fb.Cm_y = R_opt(3);
fb.dCm_dalpha_per_rad = dCm_dalpha_per_rad;
fb.detail = detail;
fb.objective = fval;
fb.exitflag = exitflag;
fb.output = output;
fb.bounds = bounds;
fb.options = options;
fb.force_balance_ok = all(isfinite(R_opt(1:2))) && norm(R_opt(1:2)) < getfield_default_local(options,'force_residual_tol',0.02);
fb.vlm_converged = isfield(detail,'converged') && detail.converged;
fb.summary_table = make_force_balance_table_local(fb);
end

% =====================================================================
function T = make_force_balance_table_local(fb)
V_mps = fb.V_mps;
alpha_deg = fb.alpha_deg;
gamma_deg = fb.gamma_deg;
theta_deg = fb.theta_deg;
u_forward_mps = fb.u_forward_mps;
w_down_mps = fb.w_down_mps;
R_L = fb.residual(1);
R_D = fb.residual(2);
Cm_y = fb.Cm_y;
force_residual_norm = fb.force_residual_norm;
dCm_dalpha = fb.dCm_dalpha_per_rad;
CL = fb.detail.CL_wind;
CD = fb.detail.CD_wind;
LD = fb.detail.LD;
L_over_W = fb.detail.L_over_W;
D_over_W = fb.detail.D_over_W;
vlm_converged = fb.vlm_converged;
solver_err = fb.detail.solver_err;
exitflag = fb.exitflag;
T = table(V_mps, alpha_deg, gamma_deg, theta_deg, u_forward_mps, w_down_mps, ...
          R_L, R_D, Cm_y, force_residual_norm, dCm_dalpha, CL, CD, LD, ...
          L_over_W, D_over_W, vlm_converged, solver_err, exitflag);
end

% =====================================================================
function y = to_unconstrained_local(x, lb, ub)
s = (x - lb) ./ max(ub - lb, 1e-12);
s = min(max(s, 1e-8), 1-1e-8);
y = log(s ./ (1-s));
end

% =====================================================================
function x = from_unconstrained_local(y, lb, ub)
y = reshape(y, 1, 2);
s = 1 ./ (1 + exp(-y));
x = lb + (ub - lb) .* s;
end

% =====================================================================
function v = getfield_default_local(s, field, default)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end

% =====================================================================
function y = finite_or_large_local(x)
y = x;
y(~isfinite(y)) = 1e3;
end
