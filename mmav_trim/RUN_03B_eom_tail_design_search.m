%% RUN_03B_EOM_TAIL_DESIGN_SEARCH
% Free tail design search using the EOM-based glide trim solver.
%
% All baseline settings are read from MMAV_USER_CONFIG.m.  Search variables
% intentionally override only CG and selected tail dimensions.

clear; clc; close all;
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(project_dir));

fprintf('\n============================================================\n');
fprintf('RUN 03B: EOM-based free tail design search\n');
fprintf('============================================================\n');

U = MMAV_USER_CONFIG();
[~, vlm_cfg, layout] = mmav_configure_case(U);
mmav_print_vlm_polar_info(vlm_cfg);
base_cfg = mmav_default_coarse_config([layout.cg_B_mm 0 0]);
base_tail_area_mm2 = base_cfg.curled_tail.span_mm * base_cfg.curled_tail.arc_chord_mm;

S = U.search;
Scr = U.screen_eom;
bounds = U.eom.bounds;
solver_options = U.eom.solver_options;
z0_default = U.eom.z0;

fprintf('\nShared config file : MMAV_USER_CONFIG.m\n');
fprintf('Tail aero mode     : %s\n', U.tail.aero_mode);
fprintf('Frame reference    : selected opt0615 CG_B = %.2f mm, wing root LE_B = %.2f mm\n', layout.cg_B_mm, layout.wing_rootLE_B_mm);
fprintf('Grid size: %d CG x %d span x %d arc x %d curl x %d incidence = %d cases\n', ...
    numel(S.cg_x_B_vec_mm), numel(S.tail_span_vec_mm), numel(S.tail_arc_chord_vec_mm), ...
    numel(S.tail_curl_vec_deg), numel(S.tail_incidence_vec_deg), ...
    numel(S.cg_x_B_vec_mm)*numel(S.tail_span_vec_mm)*numel(S.tail_arc_chord_vec_mm)*numel(S.tail_curl_vec_deg)*numel(S.tail_incidence_vec_deg));

Rows = table();
case_id = 0;
last_good_z = z0_default;
for cg_x = reshape(S.cg_x_B_vec_mm,1,[])
    for span_mm = reshape(S.tail_span_vec_mm,1,[])
        for arc_mm = reshape(S.tail_arc_chord_vec_mm,1,[])
            for curl_deg = reshape(S.tail_curl_vec_deg,1,[])
                for inc_deg = reshape(S.tail_incidence_vec_deg,1,[])
                    case_id = case_id + 1;
                    fprintf('  case %03d: CG %.1f, span %.1f, arc %.1f, curl %.1f, inc %.1f...\n', ...
                        case_id, cg_x, span_mm, arc_mm, curl_deg, inc_deg);

                    tail = U.tail;
                    tail.span_mm = span_mm;
                    tail.arc_chord_mm = arc_mm;
                    tail.curl_deg = curl_deg;
                    tail.incidence_deg = inc_deg;
                    [cfg,~,~] = mmav_configure_case(U, [cg_x 0 0], tail);
                    gap = mmav_tail_wing_gap_info(cfg);

                    try
                        trim = mmav_solve_steady_glide_trim(cfg, vlm_cfg, [cg_x 0 0], last_good_z, bounds, solver_options);
                        if isfinite(trim.residual_norm) && trim.residual_norm < 0.05
                            last_good_z = trim.z;
                        end
                    catch ME
                        warning('EOM trim solve failed: %s', ME.message);
                        trim = empty_trim_result(z0_default);
                    end

                    area_scale = (cfg.curled_tail.span_mm * cfg.curled_tail.arc_chord_mm) / base_tail_area_mm2;
                    force_residual_max = max(abs(trim.residual(1:2)));
                    passes = isfinite(trim.residual_norm) && ...
                        trim.residual_norm <= Scr.max_residual_norm && ...
                        abs(trim.residual(3)) <= Scr.max_abs_Cm && ...
                        force_residual_max <= Scr.max_force_residual && ...
                        trim.dCm_dalpha_per_rad <= Scr.max_dCm_dalpha && ...
                        trim.detail.LD >= Scr.min_LD && ...
                        gap.overlap_each_side_mm <= Scr.max_tail_overlap_mm && ...
                        trim.detail.converged;

                    trim_cost = trim.residual_norm / Scr.max_residual_norm;
                    stab_penalty = max(0, (trim.dCm_dalpha_per_rad - Scr.max_dCm_dalpha) / 0.10);
                    ld_penalty = max(0, (Scr.min_LD - trim.detail.LD) / 0.50);
                    size_penalty = 0.05 * area_scale;
                    objective = trim_cost + 2*stab_penalty + 2*ld_penalty + size_penalty;
                    if ~isfinite(objective), objective = Inf; end

                    row = table(case_id, string(cfg.curled_tail.aero_mode), cg_x, cfg.curled_tail.span_mm, cfg.curled_tail.arc_chord_mm, area_scale, ...
                        curl_deg, inc_deg, trim.V_mps, trim.alpha_deg, trim.gamma_deg, trim.theta_deg, ...
                        trim.u_forward_mps, trim.w_down_mps, trim.residual(1), trim.residual(2), trim.residual(3), ...
                        trim.residual_norm, trim.dCm_dalpha_per_rad, trim.detail.CL_wind, trim.detail.CD_wind, trim.detail.LD, ...
                        gap.clearance_each_side_mm, gap.overlap_each_side_mm, trim.detail.converged, trim.detail.solver_err, passes, objective, ...
                        'VariableNames', {'case_id','tail_aero_mode','cg_x_B_mm','tail_span_mm','tail_arc_chord_mm','tail_area_scale', ...
                        'curl_deg','incidence_deg','V_trim_mps','alpha_trim_deg','gamma_trim_deg','theta_trim_deg', ...
                        'u_forward_mps','w_down_mps','R_lift','R_drag','R_Cm','residual_norm','dCm_dalpha', ...
                        'CL_trim','CD_trim','LD_trim','tail_clearance_mm','tail_overlap_mm','vlm_converged','vlm_solver_err','passes_screen','objective'});
                    Rows = [Rows; row]; %#ok<AGROW>
                end
            end
        end
    end
end

Rows = sortrows(Rows, 'objective', 'ascend');

fprintf('\n--- Top ranked EOM-trim cases ---\n');
disp(Rows(1:min(20,height(Rows)),:));

Pass = Rows(Rows.passes_screen,:);
fprintf('\nNumber of cases passing screen: %d / %d\n', height(Pass), height(Rows));
if ~isempty(Pass)
    fprintf('\n--- Passing EOM-trim cases sorted by objective ---\n');
    disp(Pass(1:min(20,height(Pass)),:));
end

figure('Name','RUN 03B: EOM trim residual','Color','w');
scatter(Rows.cg_x_B_mm, Rows.tail_arc_chord_mm, 80, Rows.residual_norm, 'filled'); grid on; colorbar;
xlabel('active CG x_B [mm]'); ylabel('tail arc chord [mm]'); title('EOM trim residual norm');

figure('Name','RUN 03B: stability vs L/D','Color','w');
scatter(Rows.dCm_dalpha, Rows.LD_trim, 80, Rows.tail_area_scale, 'filled'); grid on; colorbar;
xline(Scr.max_dCm_dalpha,'k--','stability screen'); yline(Scr.min_LD,'k--','L/D screen');
xlabel('dC_m/d\alpha [1/rad]'); ylabel('L/D at trim'); title('EOM-trim candidate quality');

function trim = empty_trim_result(z)
trim = struct();
trim.z = z;
trim.V_mps = NaN;
trim.alpha_deg = NaN;
trim.gamma_deg = NaN;
trim.theta_deg = NaN;
trim.u_forward_mps = NaN;
trim.w_down_mps = NaN;
trim.residual = [Inf; Inf; Inf];
trim.residual_norm = Inf;
trim.dCm_dalpha_per_rad = NaN;
trim.detail = struct('CL_wind',NaN,'CD_wind',NaN,'LD',NaN,'converged',false,'solver_err',NaN);
end
