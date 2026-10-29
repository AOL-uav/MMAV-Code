function [cfg, vlm_cfg, layout] = mmav_configure_case(U, active_cg_B_mm, tail_override)
%MMAV_CONFIGURE_CASE Build cfg/vlm_cfg from the shared MMAV_USER_CONFIG struct.
%
% [cfg, vlm_cfg, layout] = mmav_configure_case(U)
% [cfg, vlm_cfg, layout] = mmav_configure_case(U, active_cg_B_mm)
% [cfg, vlm_cfg, layout] = mmav_configure_case(U, active_cg_B_mm, tail_override)
%
% This is the small glue layer that makes every RUN script use the same
% selected configuration unless a search loop intentionally overrides CG or
% tail dimensions.

if nargin < 1 || isempty(U)
    U = MMAV_USER_CONFIG();
end
if nargin < 2 || isempty(active_cg_B_mm)
    active_cg_B_mm = U.active_cg_B_mm;
end
if nargin < 3 || isempty(tail_override)
    tail = U.tail;
else
    tail = merge_structs(U.tail, tail_override);
end

layout = mmav_opt0615_layout_reference();
cfg = mmav_default_coarse_config(active_cg_B_mm);

% Optional wing-control geometry used when incidence controls are enabled.
if isfield(U,'controls') && isstruct(U.controls)
    if ~isfield(cfg.wing,'control') || ~isstruct(cfg.wing.control)
        cfg.wing.control = struct();
    end
    if isfield(U.controls,'wing_hinge_x_B_mm')
        cfg.wing.control.hinge_x_B_mm = U.controls.wing_hinge_x_B_mm;
    end
    if isfield(U.controls,'wing_hinge_z_B_mm')
        cfg.wing.control.hinge_z_B_mm = U.controls.wing_hinge_z_B_mm;
    end
end

cfg = mmav_set_tail_design(cfg, tail);

% Optional centralized fuselage-drag override.
if isfield(U,'fuselage_drag') && isstruct(U.fuselage_drag)
    flds = fieldnames(U.fuselage_drag);
    for k = 1:numel(flds)
        cfg.fuselage_drag.(flds{k}) = U.fuselage_drag.(flds{k});
    end
end

vlm_cfg = mmav_make_vlm_config(U.polar_file);
if isfield(U,'vlm_cfg') && isstruct(U.vlm_cfg)
    vlm_cfg = merge_structs(vlm_cfg, U.vlm_cfg);
end
end

% =====================================================================
function out = merge_structs(base, over)
out = base;
if isempty(over), return; end
flds = fieldnames(over);
for k = 1:numel(flds)
    f = flds{k};
    if isstruct(over.(f)) && isfield(out,f) && isstruct(out.(f))
        out.(f) = merge_structs(out.(f), over.(f));
    else
        out.(f) = over.(f);
    end
end
end
