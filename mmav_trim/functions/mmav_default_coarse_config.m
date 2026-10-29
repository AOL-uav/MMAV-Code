function cfg = mmav_default_coarse_config(user_cg_B_mm)
%MMAV_DEFAULT_COARSE_CONFIG Default coarse vehicle configuration.
%
% cfg = mmav_default_coarse_config(user_cg_B_mm)
%
% If user_cg_B_mm is provided as [x y z], the active CG is manually set to
% that body-frame point. If it is empty or omitted, the active CG is computed
% from the coarse mass components.
%
% Body/build frame B:
%   x_B = aft, y_B = right wing, z_B = up
%   origin = fuselage nose centerline

if nargin < 1
    user_cg_B_mm = [];
end

cfg = struct();
cfg.frames.B.name = 'B';
cfg.frames.B.axes = 'x aft, y right, z up';
cfg.frames.B.origin_note = 'fuselage nose centerline';

%% -------------------- Coarse mass components ---------------------
% These are intentionally coarse. Update these three entries as the design
% estimate improves. Battery/spar/servo/ballast are not separated here.

cfg.mass.fuselage.enabled = true;
cfg.mass.fuselage.name = 'fuselage';
cfg.mass.fuselage.mass_g = 100.0;
cfg.mass.fuselage.r_cg_B_mm = [35.0 0.0 0.0];

cfg.mass.wing.enabled = true;
cfg.mass.wing.name = 'wing';
cfg.mass.wing.mass_g = 34.0;
% Approximate whole wing assembly CG in the fuselage/build frame.
% This should be updated from CAD or from the previous layout model.
cfg.mass.wing.r_cg_B_mm = [72.0 0.0 0.0];

cfg.mass.curled_tail.enabled = false;
cfg.mass.curled_tail.name = 'curled_tail';
cfg.mass.curled_tail.mass_g = 12.0;
cfg.mass.curled_tail.r_cg_B_mm = [78.0 0.0 0.0];

%% -------------------- Active CG control --------------------------
if isempty(user_cg_B_mm)
    cfg.cg.mode = 'from_components';
    cfg.cg.r_cg_B_mm = [NaN NaN NaN];
else
    validateattributes(user_cg_B_mm, {'numeric'}, {'vector','numel',3,'finite'});
    cfg.cg.mode = 'manual';
    cfg.cg.r_cg_B_mm = reshape(user_cg_B_mm, 1, 3);
end

%% -------------------- Wing aero surface --------------------------
% This builds the exposed left/right wing panels directly in body frame.
% The wing is one mass component, but many aero panels.
cfg.wing.enabled = true;
cfg.wing.name = 'wing';

% Geometry source.
%   'opt0615_optimized' : reconstruct the optimized wing from opt_0615/cg.m
%   'trapezoid'         : simple root/tip trapezoid fallback
cfg.wing.source = 'opt0615_optimized';

% Root-LE location of the unshifted optimized wing in the body/build frame.
% In cg.m, x is measured from the old root LE.  The body-frame origin
% here is the fuselage nose.  The helper below converts the selected
% opt_0615 wing offset into body-frame coordinates.
layout_ref = mmav_opt0615_layout_reference();
cfg.wing.origin_B_mm = [layout_ref.wing_rootLE_B_mm 0.0 0.0];

% Fallback trapezoid values and reference values.  These are still used by
% old/simple demos if cfg.wing.source is changed to 'trapezoid'.
cfg.wing.span_mm = 800.0;
cfg.wing.root_chord_mm = 183.1;
cfg.wing.tip_chord_mm = 64.1;
cfg.wing.y_attach_mm = 40.0;            % exposed panel starts outside fuselage
cfg.wing.sweepLE_deg = 0.0;
cfg.wing.dihedral_deg = 0.0;
cfg.wing.incidence_deg = 0.0;           % positive rotates LE up / AoA up convention for geometry only
cfg.wing.Nx = 3;
cfg.wing.Ny_half = 11;

% Optimized wing reconstruction from opt_0615/cg.m.
cfg.wing.opt0615 = struct();
cfg.wing.opt0615.x_final = [1.0000; 1.0000; 0.8538; 0.6225; 0.3500; ...
                            -1.5000; 1.9007; 1.9882; 0.0202; -0.3144; 0.1250];
cfg.wing.opt0615.c0_m = 0.15;
cfg.wing.opt0615.AR = 16/3;
cfg.wing.opt0615.Nch = 5;
cfg.wing.opt0615.Ntw = 4;
cfg.wing.opt0615.dihedral_deg = 0;
cfg.wing.opt0615.plan_interp = 'pchip';
cfg.wing.opt0615.twist_interp = 'pchip';
cfg.wing.opt0615.cg_ref = 'mac';

%% -------------------- Curled-tail aero surface -------------------
% This is a centerline curled plate attached to the fuselage. The mass may
% be included in fuselage unless cfg.mass.curled_tail.enabled=true.
cfg.curled_tail.enabled = true;
cfg.curled_tail.name = 'curled_tail';
cfg.curled_tail.origin_B_mm = [70.0 0.0 5.0];   % leading-edge/attachment center
cfg.curled_tail.span_mm = 85.0;
cfg.curled_tail.arc_chord_mm = 55.0;
cfg.curled_tail.incidence_deg = 0.0;
cfg.curled_tail.curl_deg = 55.0;        % positive curls upward in +z_B
cfg.curled_tail.Nx = 6;                 % chord/arcwise panels
cfg.curled_tail.Ny = 6;                 % spanwise panels

% Aerodynamic interpretation of the fuselage-attached tail.
%   'drag_only'           : recommended physical model; tail is excluded from VLM.
%   'vlm_lifting_surface' : legacy/diagnostic model; tail is included in VLM.
cfg.curled_tail.aero_mode = 'drag_only';
cfg.curled_tail.drag_CD = 1.20;
cfg.curled_tail.drag_area_model = 'projected_normal';
cfg.curled_tail.drag_scale = 1.0;

%% -------------------- Fuselage flat-plate drag body --------------
cfg.fuselage_drag.enabled = true;
cfg.fuselage_drag.name = 'fuselage_drag';
cfg.fuselage_drag.S_m2 = 0.0038;
cfg.fuselage_drag.CD = 1.10;
cfg.fuselage_drag.r_cp_B_mm = [35.0 0.0 0.0];

end
