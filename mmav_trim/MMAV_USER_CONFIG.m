function U = MMAV_USER_CONFIG()
%MMAV_USER_CONFIG Central user-editable configuration for all RUN scripts.
%
% Edit this file first.  RUN_01, RUN_02, RUN_02B, RUN_03, and RUN_03B all
% read this same configuration.  RUN_01 does not write a hidden state file;
% instead, the shared configuration is the single source of truth.
%
% Body/build frame B:
%   x_B aft, y_B right, z_B up
%   origin = fuselage nose centerline
%
% Tail aero modes:
%   'drag_only'           : physically preferred for fuselage-attached curled tail.
%                           Tail is excluded from VLM; panel drag is applied along
%                           the relative-wind/drag direction using projected area.
%   'vlm_lifting_surface' : diagnostic/legacy mode. Tail is included in coupled VLM.
%                           This can overestimate authority for a fuselage-attached tail.

layout = mmav_opt0615_layout_reference();

U = struct();

%% -------------------- Global aero / polar -------------------------
U.polar_file = '';              % full path recommended for final runs
U.rho_kgpm3 = 1.225;
U.V_mps = 5.8;
U.alpha_vec_deg = (-2:0.5:12).';

%% -------------------- Active CG -----------------------------------
% Selected opt_0615 layout CG is layout.cg_B_mm ~= 35.52 mm.
% Use a manual CG here to test trim sensitivity.
U.active_cg_B_mm = [35 0 -10];

%% -------------------- Fuselage-attached tail ----------------------
U.tail = struct();
U.tail.enabled = true;
U.tail.aero_mode = 'drag_only';     % recommended physical model
U.tail.origin_B_mm = [70 0 5];      % attachment / leading-edge center [mm]
U.tail.span_mm = 74;                % keep inside |y| < wing attach gap
U.tail.arc_chord_mm = 130;          % arc length of curled plate [mm]
U.tail.curl_deg = 0;               % positive curls upward into +z_B
U.tail.incidence_deg = 0;
U.tail.Nx = 6;
U.tail.Ny = 6;
U.tail.enforce_gap = true;
U.tail.gap_clearance_mm = 3;

% Drag-only tail model.  This is used only when aero_mode='drag_only'.
U.tail.drag_CD = 1.30;              % broadside curved/flat-plate drag coefficient
U.tail.drag_area_model = 'projected_normal';
U.tail.drag_scale = 1.0;

%% -------------------- EOM trim solver -----------------------------
U.eom.z0 = [5.8, 5.2, 13.0];        % [V_mps, alpha_deg, gamma_deg]
U.eom.bounds.lb = [3.5, -4.0, 0.0];
U.eom.bounds.ub = [10.0, 14.0, 35.0];
U.eom.solver_options.rho_kgpm3 = U.rho_kgpm3;
U.eom.solver_options.residual_weights = [1; 1; 5];
U.eom.solver_options.max_iter = 150;
U.eom.solver_options.max_fun_evals = 260;
U.eom.solver_options.stability_dalpha_deg = 0.25;
U.eom.velocity_sweep_V_mps = (6.0:0.25:10.0).';


%% -------------------- Control trim settings -----------------------
% Wing incidence controls are applied about a spanwise hinge line.  The
% hinge x value below corresponds approximately to the opt_0615 fuselage
% center / spar axis in the B frame.
U.controls = struct();
U.controls.wing_hinge_x_B_mm = 35.0;
U.controls.wing_hinge_z_B_mm = 0.0;

% Conventional fixed-speed trim with controls:
%   x = [alpha_deg, gamma_deg, wing_collective_deg, tail_curl_delta_deg]
% Differential wing incidence is held at zero in RUN_02D because this is a
% symmetric longitudinal trim.  The aero layer supports R/L incidence for a
% later full 6-DOF trim.
U.controlled_trim = struct();
U.controlled_trim.V_mps = U.V_mps;
U.controlled_trim.x0 = [6.241792637, 11.02640606, 0.0, 40.0];
U.controlled_trim.bounds.lb = [-4.0, 0.0,  -25.0, -60.0];
U.controlled_trim.bounds.ub = [14.0, 35.0,  25.0,  60.0];
U.controlled_trim.solver_options = U.eom.solver_options;
U.controlled_trim.solver_options.residual_weights = [1; 1; 20];
U.controlled_trim.solver_options.control_penalty = 0;
U.controlled_trim.solver_options.control_scale_deg = [25; 60];
U.controlled_trim.solver_options.diagnostics_controlled_trim = false;
U.controlled_trim.solver_options.diagnostics_print_every = 10;

%% -------------------- Design search grid --------------------------
% Keep the grid modest first; EOM trim calls are expensive.
U.search.cg_x_B_vec_mm = (38:2:48).';
U.search.tail_span_vec_mm = 74;
U.search.tail_arc_chord_vec_mm = [55 82.5 110 137.5 165];
U.search.tail_curl_vec_deg = [35 45];
U.search.tail_incidence_vec_deg = 0;

%% -------------------- Fixed-speed screen --------------------------
U.screen_fixed.max_abs_Cm = 0.010;
U.screen_fixed.max_dCm_dalpha = -0.100;
U.screen_fixed.min_LD = 4.00;
U.screen_fixed.min_converged_fraction = 0.90;
U.screen_fixed.max_tail_overlap_mm = 0.0;

%% -------------------- EOM screen ----------------------------------
U.screen_eom.max_residual_norm = 0.020;
U.screen_eom.max_abs_Cm = 0.010;
U.screen_eom.max_force_residual = 0.020;
U.screen_eom.max_dCm_dalpha = -0.100;
U.screen_eom.min_LD = 4.00;
U.screen_eom.max_tail_overlap_mm = 0.0;

%% -------------------- Reference notes -----------------------------
U.reference.selected_layout_cg_B_mm = layout.cg_B_mm;
U.reference.wing_rootLE_B_mm = layout.wing_rootLE_B_mm;
U.reference.note = 'All RUN scripts read this same MMAV_USER_CONFIG.m file.';
end
