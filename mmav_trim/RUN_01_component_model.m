%% RUN_01_COMPONENT_MODEL
% Component-wise mass/geometry model for the fuselage + optimized wing + tail.
%
% This script reads the shared configuration in MMAV_USER_CONFIG.m.  The
% selected configuration is not stored by RUN_01; RUN_02/RUN_03 read the same
% configuration file independently so all scripts remain reproducible.

clear; clc; close all;
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(project_dir));

fprintf('\n============================================================\n');
fprintf('RUN 01: Component-wise MMAV model\n');
fprintf('============================================================\n');

U = MMAV_USER_CONFIG();
[cfg, vlm_cfg, layout] = mmav_configure_case(U);
vehicle = mmav_build_coarse_vehicle(cfg);
gap = mmav_tail_wing_gap_info(cfg);

fprintf('\nShared config file : MMAV_USER_CONFIG.m\n');
fprintf('Tail aero mode     : %s\n', cfg.curled_tail.aero_mode);
fprintf('Frame B            : x aft, y right, z up; origin = fuselage nose centerline.\n');
fprintf('Frame conversion   : x_B = x_opt0615_global - %.2f mm\n', layout.fuselage_nose_global_mm);
fprintf('Wing root LE_B     : %.2f mm\n', layout.wing_rootLE_B_mm);
fprintf('Selected opt0615 CG_B : %.2f mm\n', layout.cg_B_mm);
fprintf('Active CG_B        : [%.2f %.2f %.2f] mm\n', cfg.cg.r_cg_B_mm);

fprintf('\n--- Coarse mass components ---\n');
disp(vehicle.tables.mass_components);

fprintf('\n--- Aero surfaces / geometry panels ---\n');
disp(vehicle.tables.aero_surfaces);

fprintf('\n--- Fuselage drag body ---\n');
disp(vehicle.tables.aero_bodies);

fprintf('\n--- Tail design ---\n');
fprintf('%s\n', cfg.curled_tail.design_note);
fprintf('drag_CD = %.3f, drag_area_model = %s\n', cfg.curled_tail.drag_CD, cfg.curled_tail.drag_area_model);
fprintf('actual area scale relative to original baseline tail = %.3f\n', cfg.curled_tail.actual_area_scale_relative_to_original);

fprintf('\n--- Tail/wing gap diagnostic ---\n');
fprintf('wing exposed starts at |y| = %.2f mm\n', gap.wing_y_attach_mm);
fprintf('tail span = %.2f mm, halfspan = %.2f mm\n', gap.tail_span_mm, gap.tail_halfspan_mm);
fprintf('tail clearance each side = %.2f mm\n', gap.clearance_each_side_mm);
fprintf('tail overlap each side   = %.2f mm\n', gap.overlap_each_side_mm);

fprintf('\n--- VLM polar source that later RUN scripts will use ---\n');
mmav_print_vlm_polar_info(vlm_cfg);

fprintf('\n--- First 12 panels ---\n');
disp(mmav_panel_table(vehicle.panels, 12));

mmav_plot_coarse_vehicle(vehicle);
