function row = mmav_evaluate_candidate_point_vlm_safe_tail(cg_x_mm, tail_area_scale, tail_curl_deg, tail_incidence_deg, V_mps, rho_kgpm3, alpha_vec_deg, vlm_cfg, clearance_mm)
%MMAV_EVALUATE_CANDIDATE_POINT_VLM_SAFE_TAIL Candidate row using safe-span tail scaling.

if nargin < 9 || isempty(clearance_mm), clearance_mm = 3.0; end
if nargin < 8 || isempty(vlm_cfg), vlm_cfg = mmav_make_vlm_config(); end
if nargin < 7 || isempty(alpha_vec_deg), alpha_vec_deg = linspace(-2,12,29); end
if nargin < 6 || isempty(rho_kgpm3), rho_kgpm3 = 1.225; end
if nargin < 5 || isempty(V_mps), V_mps = 5.8; end
if nargin < 4 || isempty(tail_incidence_deg), tail_incidence_deg = 0; end

base_cfg = mmav_default_coarse_config([cg_x_mm 0 0]);
cfg = mmav_apply_tail_design_safe(base_cfg, tail_area_scale, tail_curl_deg, tail_incidence_deg, clearance_mm);
cfg.cg.mode = 'manual';
cfg.cg.r_cg_B_mm = [cg_x_mm 0 0];

m = mmav_vlm_passive_trim_metrics(cfg, vlm_cfg, [cg_x_mm 0 0], V_mps, rho_kgpm3, alpha_vec_deg);

gap = mmav_tail_wing_gap_info(cfg);
if tail_area_scale <= 0
    actual_area_scale = 0;
else
    actual_area_scale = getfield_default(cfg.curled_tail, 'actual_area_scale_relative_to_original', NaN);
end

row = table(cg_x_mm, tail_area_scale, actual_area_scale, tail_curl_deg, tail_incidence_deg, V_mps, ...
    m.alpha_lift_deg, m.Cm_at_lift, m.dCm_dalpha_lift_per_rad, ...
    m.LD_at_lift, m.glide_gamma_deg, m.alpha_moment_deg, m.Fz_over_W_at_moment, ...
    mean(m.converged,'omitnan'), max(m.err,[],'omitnan'), ...
    gap.tail_span_mm, gap.clearance_each_side_mm, gap.overlap_each_side_mm, ...
    'VariableNames', {'cg_x_mm','tail_area_scale_cmd','tail_area_scale_actual_orig','curl_deg','incidence_deg','V_mps', ...
    'alpha_lift_deg','Cm_at_lift','dCm_dalpha','LD_lift','gamma_deg','alpha_moment_deg','FzW_at_moment', ...
    'converged_fraction','max_solver_err','tail_span_mm','tail_clearance_mm','tail_overlap_mm'});
end

function v = getfield_default(s, field, default)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end
