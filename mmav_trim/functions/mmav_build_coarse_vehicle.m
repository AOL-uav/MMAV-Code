function vehicle = mmav_build_coarse_vehicle(cfg)
%MMAV_BUILD_COARSE_VEHICLE Build coarse mass model and panelized aero geometry.
%
% vehicle = mmav_build_coarse_vehicle(cfg)
%
% Output contains:
%   vehicle.mass_components      coarse CG/mass components
%   vehicle.panels               wing + curled-tail panels in body frame B
%   vehicle.aero_bodies          fuselage drag body in body frame B
%   vehicle.cg.r_active_B_m      active CG used for moment aggregation
%   vehicle.cg.r_from_components_B_m  computed CG from coarse mass components

vehicle = struct();
vehicle.frames = cfg.frames;

%% -------------------- Mass components -----------------------------
mass_components = struct('name',{},'enabled',{},'mass_kg',{},'r_cg_B_m',{});

names = fieldnames(cfg.mass);
for i = 1:numel(names)
    c = cfg.mass.(names{i});
    if ~isfield(c,'enabled') || c.enabled
        mass_components(end+1) = struct( ... %#ok<AGROW>
            'name', c.name, ...
            'enabled', true, ...
            'mass_kg', c.mass_g * 1e-3, ...
            'r_cg_B_m', c.r_cg_B_mm(:).' * 1e-3);
    end
end

[r_comp_cg_B_m, total_mass_kg, mass_table] = mmav_compute_total_cg(mass_components);

vehicle.mass_components = mass_components;
vehicle.mass.total_mass_kg = total_mass_kg;
vehicle.cg.r_from_components_B_m = r_comp_cg_B_m;
vehicle.tables.mass_components = mass_table;

switch lower(cfg.cg.mode)
    case 'manual'
        vehicle.cg.mode = 'manual';
        vehicle.cg.r_active_B_m = cfg.cg.r_cg_B_mm(:).' * 1e-3;
    case 'from_components'
        vehicle.cg.mode = 'from_components';
        vehicle.cg.r_active_B_m = r_comp_cg_B_m;
    otherwise
        error('Unknown cfg.cg.mode: %s', cfg.cg.mode);
end

%% -------------------- Panelized aero surfaces ---------------------
panels = struct([]);

wing_meta = struct();
if isfield(cfg,'wing') && cfg.wing.enabled
    wing_source = getfield_default(cfg.wing, 'source', 'trapezoid');
    switch lower(char(string(wing_source)))
        case {'opt0615_optimized','opt_0615_optimized','optimized'}
            [wing_panels, wing_meta] = mmav_make_opt0615_optimized_wing_panels(cfg.wing);
        case {'trapezoid','simple'}
            wing_panels = mmav_make_wing_panels(cfg.wing);
            wing_meta = struct('source','trapezoid');
        otherwise
            error('Unknown cfg.wing.source: %s', wing_source);
    end
    panels = append_panels(panels, wing_panels);
else
    wing_panels = struct([]);
end

if isfield(cfg,'curled_tail') && cfg.curled_tail.enabled
    tail_panels = mmav_make_curled_tail_panels(cfg.curled_tail);
    panels = append_panels(panels, tail_panels);
else
    tail_panels = struct([]);
end

% Add global panel id.
for k = 1:numel(panels)
    panels(k).id = k;
end
vehicle.panels = panels;
vehicle.wing_panels = wing_panels;
vehicle.tail_panels = tail_panels;
vehicle.wing_meta = wing_meta;

%% -------------------- Fuselage drag body --------------------------
aero_bodies = struct('name',{},'enabled',{},'S_m2',{},'CD',{},'r_ref_B_m',{}, ...
                     'F_B_N',{},'M_ref_B_Nm',{});
if isfield(cfg,'fuselage_drag') && cfg.fuselage_drag.enabled
    b = cfg.fuselage_drag;
    aero_bodies(end+1) = struct( ...
        'name', b.name, ...
        'enabled', true, ...
        'S_m2', b.S_m2, ...
        'CD', b.CD, ...
        'r_ref_B_m', b.r_cp_B_mm(:).' * 1e-3, ...
        'F_B_N', [0 0 0], ...
        'M_ref_B_Nm', [0 0 0]);
end
vehicle.aero_bodies = aero_bodies;

%% -------------------- References ---------------------------------
S_wing = sum_panel_area_by_component(panels, 'wing');
if S_wing <= 0
    S_wing = sum([panels.area_m2]);
end

if isfield(wing_meta,'MAC_m') && isfinite(wing_meta.MAC_m)
    cref_m = wing_meta.MAC_m;
else
    lambda = cfg.wing.tip_chord_mm / max(cfg.wing.root_chord_mm, 1e-12);
    cref_m = (2/3) * (cfg.wing.root_chord_mm*1e-3) * ...
        (1 + lambda + lambda^2) / max(1 + lambda, 1e-12);
end

if isfield(wing_meta,'bref_m') && isfinite(wing_meta.bref_m)
    bref_m = wing_meta.bref_m;
else
    bref_m = cfg.wing.span_mm * 1e-3;
end

vehicle.refs.Sref_m2 = S_wing;
vehicle.refs.cref_m = cref_m;
vehicle.refs.bref_m = bref_m;
if isfield(wing_meta,'source')
    vehicle.refs.note = sprintf('Sref is exposed wing panel area; cref from %s geometry.', wing_meta.source);
else
    vehicle.refs.note = 'Sref is exposed wing panel area; cref is trapezoid MAC estimate.';
end

%% -------------------- Tables -------------------------------------
vehicle.tables.aero_surfaces = make_aero_surface_table(panels);
vehicle.tables.aero_bodies = make_aero_body_table(aero_bodies);

end


% =====================================================================
function v = getfield_default(s, field, default)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end

% =====================================================================
function panels = append_panels(panels, new_panels)
if isempty(new_panels)
    return;
end
if isempty(panels)
    panels = new_panels;
else
    panels = [panels(:); new_panels(:)]; %#ok<AGROW>
end
end

% =====================================================================
function S = sum_panel_area_by_component(panels, component_name)
S = 0;
for k = 1:numel(panels)
    if isfield(panels(k),'component') && strcmpi(panels(k).component, component_name)
        S = S + panels(k).area_m2;
    end
end
end

% =====================================================================
function T = make_aero_surface_table(panels)
if isempty(panels)
    T = table();
    return;
end

components = unique(string({panels.component}).');
name = strings(numel(components),1);
N_panels = zeros(numel(components),1);
S_m2 = zeros(numel(components),1);
x_centroid_mm = zeros(numel(components),1);
y_centroid_mm = zeros(numel(components),1);
z_centroid_mm = zeros(numel(components),1);

for i = 1:numel(components)
    comp = components(i);
    idx = strcmp(string({panels.component}), comp);
    p = panels(idx);
    A = [p.area_m2].';
    C = reshape([p.centroid_B_m], 3, []).';
    S = sum(A);
    if S > 0
        cbar = sum(C .* A, 1) / S;
    else
        cbar = [NaN NaN NaN];
    end
    name(i) = comp;
    N_panels(i) = numel(p);
    S_m2(i) = S;
    x_centroid_mm(i) = 1e3*cbar(1);
    y_centroid_mm(i) = 1e3*cbar(2);
    z_centroid_mm(i) = 1e3*cbar(3);
end

T = table(name, N_panels, S_m2, x_centroid_mm, y_centroid_mm, z_centroid_mm);
end

% =====================================================================
function T = make_aero_body_table(aero_bodies)
if isempty(aero_bodies)
    T = table();
    return;
end

name = strings(numel(aero_bodies),1);
S_m2 = zeros(numel(aero_bodies),1);
CD = zeros(numel(aero_bodies),1);
x_ref_mm = zeros(numel(aero_bodies),1);
y_ref_mm = zeros(numel(aero_bodies),1);
z_ref_mm = zeros(numel(aero_bodies),1);

for i = 1:numel(aero_bodies)
    name(i) = string(aero_bodies(i).name);
    S_m2(i) = aero_bodies(i).S_m2;
    CD(i) = aero_bodies(i).CD;
    x_ref_mm(i) = 1e3*aero_bodies(i).r_ref_B_m(1);
    y_ref_mm(i) = 1e3*aero_bodies(i).r_ref_B_m(2);
    z_ref_mm(i) = 1e3*aero_bodies(i).r_ref_B_m(3);
end

T = table(name, S_m2, CD, x_ref_mm, y_ref_mm, z_ref_mm);
end
