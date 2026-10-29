function metrics = mmav_vlm_passive_trim_metrics(cfg, vlm_cfg, user_cg_B_mm, V_mps, rho_kgpm3, alpha_vec_deg)
%MMAV_VLM_PASSIVE_TRIM_METRICS Sweep alpha with nonlinear VLM and extract trim metrics.
%
% Same purpose as mmav_passive_trim_metrics(), but using
% mmav_eval_vlm_aero_state() instead of the simple local CL/CD model.

if nargin < 6 || isempty(alpha_vec_deg), alpha_vec_deg = linspace(-4,14,37); end
if nargin < 5 || isempty(rho_kgpm3), rho_kgpm3 = 1.225; end
if nargin < 4 || isempty(V_mps), V_mps = 5.8; end
if nargin < 3, user_cg_B_mm = []; end
if nargin < 2 || isempty(vlm_cfg), vlm_cfg = mmav_default_vlm_config(); end

alpha_vec_deg = alpha_vec_deg(:);
n = numel(alpha_vec_deg);

Fz_over_W = NaN(n,1);
Cm_y = NaN(n,1);
CL_wind = NaN(n,1);
CD_wind = NaN(n,1);
CFx_body = NaN(n,1);
CFz_body = NaN(n,1);
Fx_N = NaN(n,1);
Fz_N = NaN(n,1);
My_Nm = NaN(n,1);
converged = false(n,1);
iter = NaN(n,1);
err = NaN(n,1);

W_N = NaN; Sref_m2 = NaN; cref_m = NaN; q_Pa = NaN;
warm_gamma = [];

for ii = 1:n
    state = struct('rho_kgpm3',rho_kgpm3, 'V_mps',V_mps, ...
                   'alpha_deg',alpha_vec_deg(ii), 'beta_deg',0.0);
    try
        [out, warm_gamma] = mmav_eval_vlm_aero_state(cfg, user_cg_B_mm, state, vlm_cfg, warm_gamma);
    catch ME
        warning('VLM alpha sweep failed at alpha=%.3f deg: %s', alpha_vec_deg(ii), ME.message);
        warm_gamma = [];
        continue;
    end

    W_N = out.vehicle.mass.total_mass_kg * 9.80665;
    Sref_m2 = out.Sref_m2;
    cref_m = out.cref_m;
    q_Pa = out.q_Pa;

    Fx_N(ii) = out.loads.F_B_N(1);
    Fz_N(ii) = out.loads.F_B_N(3);
    My_Nm(ii) = out.loads.M_CG_B_Nm(2);
    Fz_over_W(ii) = Fz_N(ii) / W_N;
    Cm_y(ii) = out.Cm_y;
    CL_wind(ii) = out.CL_wind;
    CD_wind(ii) = out.CD_wind;
    CFx_body(ii) = out.CF_B(1);
    CFz_body(ii) = out.CF_B(3);
    if isfield(out.vlm_result_lifting,'converged')
        converged(ii) = out.vlm_result_lifting.converged;
    end
    if isfield(out.vlm_result_lifting,'iter')
        iter(ii) = out.vlm_result_lifting.iter;
    end
    if isfield(out.vlm_result_lifting,'err')
        err(ii) = out.vlm_result_lifting.err;
    end
end

alpha_lift_deg = mmav_find_crossing(alpha_vec_deg, Fz_over_W, 1.0);
alpha_moment_deg = mmav_find_crossing(alpha_vec_deg, Cm_y, 0.0);

Cm_at_lift = interp_safe(alpha_vec_deg, Cm_y, alpha_lift_deg);
Fz_over_W_at_moment = interp_safe(alpha_vec_deg, Fz_over_W, alpha_moment_deg);
CL_at_lift = interp_safe(alpha_vec_deg, CL_wind, alpha_lift_deg);
CD_at_lift = interp_safe(alpha_vec_deg, CD_wind, alpha_lift_deg);
CFx_at_lift = interp_safe(alpha_vec_deg, CFx_body, alpha_lift_deg);
CFz_at_lift = interp_safe(alpha_vec_deg, CFz_body, alpha_lift_deg);
Fx_at_lift_N = interp_safe(alpha_vec_deg, Fx_N, alpha_lift_deg);
Fz_at_lift_N = interp_safe(alpha_vec_deg, Fz_N, alpha_lift_deg);
My_at_lift_Nm = interp_safe(alpha_vec_deg, My_Nm, alpha_lift_deg);

if all(isfinite(Cm_y)) && numel(alpha_vec_deg) >= 2
    dCm_dalpha_per_deg = gradient(Cm_y, alpha_vec_deg);
else
    dCm_dalpha_per_deg = NaN(size(Cm_y));
end
dCm_dalpha_lift_per_deg = interp_safe(alpha_vec_deg, dCm_dalpha_per_deg, alpha_lift_deg);
dCm_dalpha_lift_per_rad = dCm_dalpha_lift_per_deg * (180/pi);

if isfinite(CL_at_lift) && isfinite(CD_at_lift) && abs(CD_at_lift) > 1e-12
    LD_at_lift = CL_at_lift / CD_at_lift;
else
    LD_at_lift = NaN;
end
if isfinite(CL_at_lift) && isfinite(CD_at_lift) && abs(CL_at_lift) > 1e-12
    glide_gamma_deg = rad2deg(atan2(CD_at_lift, CL_at_lift));
else
    glide_gamma_deg = NaN;
end

out_at_lift = [];
source_table_at_lift = table();
if isfinite(alpha_lift_deg)
    state_lift = struct('rho_kgpm3',rho_kgpm3, 'V_mps',V_mps, ...
                        'alpha_deg',alpha_lift_deg, 'beta_deg',0.0);
    try
        [out_at_lift,~] = mmav_eval_vlm_aero_state(cfg, user_cg_B_mm, state_lift, vlm_cfg, []);
        source_table_at_lift = out_at_lift.loads.tables.by_source;
    catch ME
        warning('VLM lift-balance re-evaluation failed: %s', ME.message);
    end
end

metrics = struct();
metrics.model = 'VLM/nonlinear polar';
metrics.V_mps = V_mps;
metrics.rho_kgpm3 = rho_kgpm3;
metrics.q_Pa = q_Pa;
metrics.W_N = W_N;
metrics.Sref_m2 = Sref_m2;
metrics.cref_m = cref_m;
metrics.alpha_vec_deg = alpha_vec_deg;
metrics.Fz_over_W = Fz_over_W;
metrics.Cm_y = Cm_y;
metrics.CL_wind = CL_wind;
metrics.CD_wind = CD_wind;
metrics.CFx_body = CFx_body;
metrics.CFz_body = CFz_body;
metrics.Fx_N = Fx_N;
metrics.Fz_N = Fz_N;
metrics.My_Nm = My_Nm;
metrics.converged = converged;
metrics.iter = iter;
metrics.err = err;
metrics.alpha_lift_deg = alpha_lift_deg;
metrics.Cm_at_lift = Cm_at_lift;
metrics.alpha_moment_deg = alpha_moment_deg;
metrics.Fz_over_W_at_moment = Fz_over_W_at_moment;
metrics.CL_wind_at_lift = CL_at_lift;
metrics.CD_wind_at_lift = CD_at_lift;
metrics.CFx_body_at_lift = CFx_at_lift;
metrics.CFz_body_at_lift = CFz_at_lift;
metrics.Fx_at_lift_N = Fx_at_lift_N;
metrics.Fz_at_lift_N = Fz_at_lift_N;
metrics.My_at_lift_Nm = My_at_lift_Nm;
metrics.dCm_dalpha_lift_per_deg = dCm_dalpha_lift_per_deg;
metrics.dCm_dalpha_lift_per_rad = dCm_dalpha_lift_per_rad;
metrics.static_stable_lift = dCm_dalpha_lift_per_rad < 0;
metrics.LD_at_lift = LD_at_lift;
metrics.glide_gamma_deg = glide_gamma_deg;
metrics.out_at_lift = out_at_lift;
metrics.source_table_at_lift = source_table_at_lift;
end

% =====================================================================
function yi = interp_safe(x, y, xi)
if ~isfinite(xi) || isempty(x) || isempty(y) || numel(x) ~= numel(y)
    yi = NaN;
    return;
end
valid = isfinite(x) & isfinite(y);
if nnz(valid) < 2
    yi = NaN;
    return;
end
xmin = min(x(valid));
xmax = max(x(valid));
if xi < xmin || xi > xmax
    yi = NaN;
    return;
end
yi = interp1(x(valid), y(valid), xi, 'linear');
end
