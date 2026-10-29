function vlm_cfg = mmav_default_vlm_config()
%MMAV_DEFAULT_VLM_CONFIG Default settings for opt_0615 nonlinear VLM adapter.
%
% This config is used by mmav_eval_vlm_aero_state().  It keeps the coarse
% component/CG framework unchanged but replaces the simple local CL/CD force
% assignment with the opt_0615 nonlinear VLM + 2D polar solver.
%
% Notes:
%   - If a real XFOIL polar file is available, set vlm_cfg.polar_file.
%   - If no file is found, the adapter uses a synthetic polar so that the
%     geometry/trim scripts can still run.  The synthetic polar is only a
%     fallback and should not be used for final design decisions.

vlm_cfg = struct();
vlm_cfg.diagnostics_residual = true;
vlm_cfg.diagnostics_controlled_residual = true;
vlm_cfg.diagnostics_aero_state = false;

% Solver settings passed to vlm_xfoil_analyze_nonlinear_polar.m
vlm_cfg.vlm = struct();
vlm_cfg.vlm.trailing_length_factor = 50;   % wake length = factor * wing span
vlm_cfg.vlm.ref_area = 'projected_xy';     % solver internal only; final coeffs use vehicle.refs
vlm_cfg.vlm.nl.max_iter = 1200;
vlm_cfg.vlm.nl.relax    = 0.25;
vlm_cfg.vlm.nl.tol      = 2e-3;
vlm_cfg.vlm.nl.verbose  = false;

% Polar loading.
% Leave empty to auto-search common files in the current/path folders.
vlm_cfg.polar_file = '';
vlm_cfg.polar_search_files = { ...
    'NACA_2408.dat', ...
    'NACA2408.dat', ...
    'NACA_2408_polar.dat', ...
    'naca2408.dat'};
vlm_cfg.allow_synthetic_polar_if_missing = true;
vlm_cfg.synthetic_polar = struct();
vlm_cfg.synthetic_polar.alpha0_deg = -2.0;     % rough cambered-airfoil fallback
vlm_cfg.synthetic_polar.CLa_per_rad = 5.7;
vlm_cfg.synthetic_polar.CL_min = -1.25;
vlm_cfg.synthetic_polar.CL_max =  1.35;
vlm_cfg.synthetic_polar.CD0 = 0.035;
vlm_cfg.synthetic_polar.k  = 0.080;
vlm_cfg.synthetic_polar.Cm0 = -0.045;

% Optional polar-family camber/flap parameter. Empty for ordinary single polar.
vlm_cfg.camber_param = [];

% Fuselage model remains the same flat-plate drag body from the coarse cfg.
vlm_cfg.include_fuselage_drag = true;

% If true, also runs wing-only and tail-only VLM solves for a diagnostic table.
% Those component values are approximate because the actual trim result uses
% the fully coupled wing+tail VLM solve.
vlm_cfg.compute_separate_component_diagnostics = false;

% Print a warning once if synthetic polar is used.
vlm_cfg.verbose = true;

end
