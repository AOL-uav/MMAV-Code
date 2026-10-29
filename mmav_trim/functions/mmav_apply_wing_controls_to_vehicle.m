function vehicle = mmav_apply_wing_controls_to_vehicle(vehicle, cfg, state)
%MMAV_APPLY_WING_CONTROLS_TO_VEHICLE Rotate wing panels for incidence controls.
%
% Each wing panel is rotated about a spanwise hinge axis parallel to y_B.
% Positive incidence uses a +Ry rotation and is intended to increase wing AoA.
%
% Longitudinal trim normally uses:
%   wing_R_incidence = wing_L_incidence = wing_collective
%
% Roll control can later use differential incidence:
%   wing_R = collective + differential
%   wing_L = collective - differential

ctrl = mmav_get_control_from_state(state);
if ~ctrl.enabled || isempty(vehicle.panels)
    vehicle.control = ctrl;
    return;
end

hinge_x_B_mm = 35.0;
hinge_z_B_mm = 0.0;
if isfield(cfg,'wing') && isfield(cfg.wing,'control')
    if isfield(cfg.wing.control,'hinge_x_B_mm') && ~isempty(cfg.wing.control.hinge_x_B_mm)
        hinge_x_B_mm = cfg.wing.control.hinge_x_B_mm;
    end
    if isfield(cfg.wing.control,'hinge_z_B_mm') && ~isempty(cfg.wing.control.hinge_z_B_mm)
        hinge_z_B_mm = cfg.wing.control.hinge_z_B_mm;
    end
end

hinge_ref_m = [hinge_x_B_mm, 0, hinge_z_B_mm] * 1e-3;

panels = vehicle.panels;
for k = 1:numel(panels)
    if ~isfield(panels(k),'component') || ~strcmpi(panels(k).component,'wing')
        continue;
    end
    surf = string(panels(k).surface);
    if contains(lower(surf), 'wing_r')
        delta_deg = ctrl.wing_R_incidence_deg;
    elseif contains(lower(surf), 'wing_l')
        delta_deg = ctrl.wing_L_incidence_deg;
    else
        delta_deg = ctrl.wing_collective_deg;
    end
    panels(k) = rotate_panel_about_y(panels(k), hinge_ref_m, delta_deg);
end

% Rebuild panel subsets while preserving the same vehicle-level references.
vehicle.panels = panels;
keep_wing = false(numel(panels),1);
keep_tail = false(numel(panels),1);
for k = 1:numel(panels)
    keep_wing(k) = isfield(panels(k),'component') && strcmpi(panels(k).component,'wing');
    keep_tail(k) = isfield(panels(k),'component') && strcmpi(panels(k).component,'curled_tail');
end
vehicle.wing_panels = panels(keep_wing);
vehicle.tail_panels = panels(keep_tail);
vehicle.control = ctrl;
vehicle.control.wing_hinge_x_B_mm = hinge_x_B_mm;
vehicle.control.wing_hinge_z_B_mm = hinge_z_B_mm;
end

% =====================================================================
function p = rotate_panel_about_y(p, hinge_ref_m, delta_deg)
R = Ry(deg2rad(delta_deg));
p.p1_B_m = rotate_point(p.p1_B_m, hinge_ref_m, R);
p.p2_B_m = rotate_point(p.p2_B_m, hinge_ref_m, R);
p.p3_B_m = rotate_point(p.p3_B_m, hinge_ref_m, R);
p.p4_B_m = rotate_point(p.p4_B_m, hinge_ref_m, R);
p = mmav_enrich_panel_geometry(p, [0 0 1]);
end

% =====================================================================
function r2 = rotate_point(r, hinge_ref_m, R)
r_rel = r - hinge_ref_m;
r2 = hinge_ref_m + (R * r_rel(:)).';
end

% =====================================================================
function R = Ry(a)
ca = cos(a); sa = sin(a);
R = [ ca  0  sa; ...
       0  1   0; ...
      -sa 0  ca];
end
