%% RUN_03_TAIL_DESIGN_SEARCH
% Free tail design search using the fixed-speed alpha-sweep diagnostic.
%
% All baseline settings are read from MMAV_USER_CONFIG.m.  Search variables
% intentionally override only CG and selected tail dimensions.

clear; clc; close all;
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(project_dir));

fprintf('\n============================================================\n');
fprintf('RUN 03: fixed-speed free tail design search\n');
fprintf('============================================================\n');

U = MMAV_USER_CONFIG();
[~, vlm_cfg, layout] = mmav_configure_case(U);
mmav_print_vlm_polar_info(vlm_cfg);
base_cfg = mmav_default_coarse_config([layout.cg_B_mm 0 0]);
base_tail_area_mm2 = base_cfg.curled_tail.span_mm * base_cfg.curled_tail.arc_chord_mm;

S = U.search;
Scr = U.screen_fixed;

fprintf('\nShared config file : MMAV_USER_CONFIG.m\n');
fprintf('Tail aero mode     : %s\n', U.tail.aero_mode);
fprintf('Frame reference    : selected opt0615 CG_B = %.2f mm, wing root LE_B = %.2f mm\n', layout.cg_B_mm, layout.wing_rootLE_B_mm);
fprintf('Grid size: %d CG x %d span x %d arc x %d curl x %d incidence = %d cases\n', ...
    numel(S.cg_x_B_vec_mm), numel(S.tail_span_vec_mm), numel(S.tail_arc_chord_vec_mm), ...
    numel(S.tail_curl_vec_deg), numel(S.tail_incidence_vec_deg), ...
    numel(S.cg_x_B_vec_mm)*numel(S.tail_span_vec_mm)*numel(S.tail_arc_chord_vec_mm)*numel(S.tail_curl_vec_deg)*numel(S.tail_incidence_vec_deg));

Rows = table();
case_id = 0;
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
                        m = mmav_vlm_passive_trim_metrics(cfg, vlm_cfg, [cg_x 0 0], U.V_mps, U.rho_kgpm3, U.alpha_vec_deg);
                    catch ME
                        warning('Case failed: %s', ME.message);
                        m = empty_metrics();
                    end

                    area_scale = (cfg.curled_tail.span_mm * cfg.curled_tail.arc_chord_mm) / base_tail_area_mm2;
                    conv_frac = mean(m.converged,'omitnan');
                    max_err = max(m.err,[],'omitnan');

                    passes = isfinite(m.Cm_at_lift) && isfinite(m.dCm_dalpha_lift_per_rad) && isfinite(m.LD_at_lift) && ...
                        abs(m.Cm_at_lift) <= Scr.max_abs_Cm && ...
                        m.dCm_dalpha_lift_per_rad <= Scr.max_dCm_dalpha && ...
                        m.LD_at_lift >= Scr.min_LD && ...
                        gap.overlap_each_side_mm <= Scr.max_tail_overlap_mm && ...
                        conv_frac >= Scr.min_converged_fraction;

                    trim_cost = abs(m.Cm_at_lift) / Scr.max_abs_Cm;
                    stab_penalty = max(0, (m.dCm_dalpha_lift_per_rad - Scr.max_dCm_dalpha) / 0.10);
                    ld_penalty = max(0, (Scr.min_LD - m.LD_at_lift) / 0.50);
                    size_penalty = 0.05 * area_scale;
                    objective = trim_cost + 2*stab_penalty + 2*ld_penalty + size_penalty;
                    if ~isfinite(objective), objective = Inf; end

                    row = table(case_id, string(cfg.curled_tail.aero_mode), cg_x, cfg.curled_tail.span_mm, cfg.curled_tail.arc_chord_mm, area_scale, ...
                        curl_deg, inc_deg, U.V_mps, m.alpha_lift_deg, m.Cm_at_lift, m.dCm_dalpha_lift_per_rad, ...
                        m.LD_at_lift, m.glide_gamma_deg, m.alpha_moment_deg, m.Fz_over_W_at_moment, ...
                        conv_frac, max_err, gap.clearance_each_side_mm, gap.overlap_each_side_mm, passes, objective, ...
                        'VariableNames', {'case_id','tail_aero_mode','cg_x_B_mm','tail_span_mm','tail_arc_chord_mm','tail_area_scale','curl_deg','incidence_deg','V_mps', ...
                        'alpha_lift_deg','Cm_at_lift','dCm_dalpha','LD_lift','gamma_deg','alpha_moment_deg','FzW_at_moment', ...
                        'converged_fraction','max_solver_err','tail_clearance_mm','tail_overlap_mm','passes_screen','objective'});
                    Rows = [Rows; row]; %#ok<AGROW>
                end
            end
        end
    end
end

Rows = sortrows(Rows, 'objective', 'ascend');

fprintf('\n--- Top 20 ranked cases ---\n');
disp(Rows(1:min(20,height(Rows)),:));

Pass = Rows(Rows.passes_screen,:);
fprintf('\nNumber of cases passing screen: %d / %d\n', height(Pass), height(Rows));
if ~isempty(Pass)
    fprintf('\n--- Passing cases sorted by objective ---\n');
    disp(Pass(1:min(20,height(Pass)),:));
end

figure('Name','RUN 03: trim residual','Color','w');
scatter(Rows.cg_x_B_mm, Rows.tail_area_scale, 60, Rows.Cm_at_lift, 'filled'); hold on;
idx = Rows.passes_screen;
scatter(Rows.cg_x_B_mm(idx), Rows.tail_area_scale(idx), 90, 'ko', 'LineWidth',1.4);
ylabel('tail area scale [-]'); xlabel('active CG x_B [mm]'); title('C_{m,y} at F_z/W=1'); colorbar; grid on;

figure('Name','RUN 03: stability vs L/D','Color','w');
scatter(Rows.dCm_dalpha, Rows.LD_lift, 60, Rows.tail_area_scale, 'filled'); hold on;
xline(Scr.max_dCm_dalpha,'k--'); yline(Scr.min_LD,'k--'); colorbar; grid on;
xlabel('dC_{m,y}/d\alpha [1/rad]'); ylabel('L/D at F_z/W=1'); title('Tail design search');

function m = empty_metrics()
m = struct();
m.alpha_lift_deg = NaN;
m.Cm_at_lift = NaN;
m.dCm_dalpha_lift_per_rad = NaN;
m.LD_at_lift = NaN;
m.glide_gamma_deg = NaN;
m.alpha_moment_deg = NaN;
m.Fz_over_W_at_moment = NaN;
m.converged = false;
m.err = Inf;
end
