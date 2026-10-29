function trim = mmav_solve_controlled_trim_at_speed(cfg, vlm_cfg, user_cg_B_mm, V_mps, x0, bounds, options)
%MMAV_SOLVE_CONTROLLED_TRIM_AT_SPEED Conventional fixed-speed trim with controls.
%
% Unknown vector:
%   x = [alpha_deg, gamma_deg, wing_collective_deg, tail_curl_delta_deg]
%
% Residual equations:
%   L/W - cos(gamma) = 0
%   D/W - sin(gamma) = 0
%   Cm_y             = 0
%
% Since two controls are available for one pitch-moment equation, the solve is
% formulated as a weighted least-squares problem with a small control penalty.
% This chooses a small-control solution among multiple possible trim points.

if nargin < 7 || isempty(options), options = struct(); end
if nargin < 6 || isempty(bounds)
    bounds.lb = [-4.0, 0.0, -25.0, -30.0];
    bounds.ub = [14.0, 35.0, 25.0, 50.0];
end
if nargin < 5 || isempty(x0), x0 = [5.0, 13.0, 0.0, 0.0]; end
if nargin < 4 || isempty(V_mps), V_mps = 5.8; end
if nargin < 3, user_cg_B_mm = []; end
if nargin < 2 || isempty(vlm_cfg), vlm_cfg = mmav_default_vlm_config(); end

rho = getfield_default(options, 'rho_kgpm3', 1.225);
weights = getfield_default(options, 'residual_weights', [1.0; 1.0; 8.0]);
weights = reshape(weights, 3, 1);
control_penalty = getfield_default(options, 'control_penalty', 1e-4);
control_scale = getfield_default(options, 'control_scale_deg', [10.0; 20.0]);
control_scale = reshape(control_scale, 2, 1);
max_iter = getfield_default(options, 'max_iter', 250);
max_fun_evals = getfield_default(options, 'max_fun_evals', 500);
tol_x = getfield_default(options, 'tol_x', 1e-3);
tol_fun = getfield_default(options, 'tol_fun', 1e-3);
stab_dalpha_deg = getfield_default(options, 'stability_dalpha_deg', 0.05);
diag_on = getfield_default(options, 'diagnostics_controlled_trim', false);
diag_every = getfield_default(options, 'diagnostics_print_every', 10);

lb = reshape(bounds.lb, 1, 4);
ub = reshape(bounds.ub, 1, 4);
x0 = min(max(reshape(x0,1,4), lb + 1e-6), ub - 1e-6);
y0 = to_unconstrained(x0, lb, ub);

eval_count = 0;
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
        eval_count = eval_count + 1;
        x = from_unconstrained(y, lb, ub);
        [R,detail] = mmav_controlled_trim_residual_at_speed(x, cfg, vlm_cfg, user_cg_B_mm, V_mps, rho);
        if any(~isfinite(R)) || ~isfield(detail,'converged') || ~detail.converged
            J = 1e6 + nansum_safe((weights .* finite_or_large(R)).^2);
        else
            ctrl_vec = [x(3); x(4)] ./ max(control_scale, 1e-12);
            J = sum((weights .* R).^2) + control_penalty * sum(ctrl_vec.^2);
        end
        if ~isfinite(J)
            J = 1e9;
        end
        if diag_on && (eval_count == 1 || mod(eval_count, diag_every) == 0)
            fprintf(['[CTRL-TRIM eval %04d] ', ...
                'V=%.3f a=%.3f g=%.3f wingC=%+.2f tailCurl=%+.2f | ', ...
                'R=[%+.2e %+.2e %+.2e] J=%.3e | ', ...
                'L/W=%.3f D/W=%.3f Cm=%+.4f LD=%.2f vlm=%d\n'], ...
                eval_count, V_mps, x(1), x(2), x(3), x(4), ...
                R(1), R(2), R(3), J, ...
                detail.L_over_W, detail.D_over_W, detail.Cm_y, detail.LD, detail.converged);
        end
    end

opts = optimset('Display','off', ...
                'MaxIter',max_iter, ...
                'MaxFunEvals',max_fun_evals, ...
                'TolX',tol_x, ...
                'TolFun',tol_fun, ...
                'OutputFcn',@outfun);

[y_opt, fval, exitflag, output] = fminsearch(@objective_y, y0, opts);
x_opt = from_unconstrained(y_opt, lb, ub);
[R_opt, detail] = mmav_controlled_trim_residual_at_speed(x_opt, cfg, vlm_cfg, user_cg_B_mm, V_mps, rho);

% Static-stability slope at fixed controls and fixed V.
dCm_dalpha_per_rad = NaN;
try
    xp = x_opt; xm = x_opt;
    xp(1) = detail.alpha_deg + stab_dalpha_deg;
    xm(1) = detail.alpha_deg - stab_dalpha_deg;
    [~,dp] = mmav_controlled_trim_residual_at_speed(xp, cfg, vlm_cfg, user_cg_B_mm, V_mps, rho);
    [~,dm] = mmav_controlled_trim_residual_at_speed(xm, cfg, vlm_cfg, user_cg_B_mm, V_mps, rho);
    dCm_dalpha_per_deg = (dp.Cm_y - dm.Cm_y) / (2*stab_dalpha_deg);
    dCm_dalpha_per_rad = dCm_dalpha_per_deg * (180/pi);
catch ME
    warning('Controlled trim static-stability finite difference failed: %s', ME.message);
end

trim = struct();
trim.model = 'fixed-speed controlled steady-glide trim';
trim.V_mps = V_mps;
trim.x = x_opt;
trim.alpha_deg = detail.alpha_deg;
trim.gamma_deg = detail.gamma_deg;
trim.theta_deg = detail.theta_deg;
trim.u_forward_mps = detail.u_forward_mps;
trim.w_down_mps = detail.w_down_mps;
trim.wing_collective_deg = detail.wing_collective_deg;
trim.wing_differential_deg = detail.wing_differential_deg;
trim.wing_R_incidence_deg = detail.wing_R_incidence_deg;
trim.wing_L_incidence_deg = detail.wing_L_incidence_deg;
trim.tail_curl_delta_deg = detail.tail_curl_delta_deg;
trim.tail_controlled_curl_deg = detail.tail_controlled_curl_deg;
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
trim.trim_residual_ok = isfinite(trim.residual_norm) && trim.residual_norm <= getfield_default(options,'trim_residual_tol',0.02);
trim.vlm_converged = isfield(detail,'converged') && detail.converged;
trim.summary_table = make_summary_table(trim);
end

% =====================================================================
function T = make_summary_table(trim)
V_mps = trim.V_mps;
alpha_deg = trim.alpha_deg;
gamma_deg = trim.gamma_deg;
theta_deg = trim.theta_deg;
wing_collective_deg = trim.wing_collective_deg;
wing_R_incidence_deg = trim.wing_R_incidence_deg;
wing_L_incidence_deg = trim.wing_L_incidence_deg;
tail_curl_delta_deg = trim.tail_curl_delta_deg;
tail_controlled_curl_deg = trim.tail_controlled_curl_deg;
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
trim_residual_ok = trim.trim_residual_ok;
optimizer_exitflag = trim.exitflag;
solver_err = trim.detail.solver_err;
T = table(V_mps, alpha_deg, gamma_deg, theta_deg, ...
          wing_collective_deg, wing_R_incidence_deg, wing_L_incidence_deg, ...
          tail_curl_delta_deg, tail_controlled_curl_deg, ...
          R_L, R_D, R_Cm, residual_norm, Cm_y, dCm_dalpha, CL, CD, LD, ...
          L_over_W, D_over_W, vlm_converged, trim_residual_ok, solver_err, optimizer_exitflag);
end

% =====================================================================
function y = to_unconstrained(x, lb, ub)
s = (x - lb) ./ max(ub - lb, 1e-12);
s = min(max(s, 1e-8), 1-1e-8);
y = log(s ./ (1-s));
end

% =====================================================================
function x = from_unconstrained(y, lb, ub)
y = reshape(y, 1, 4);
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
