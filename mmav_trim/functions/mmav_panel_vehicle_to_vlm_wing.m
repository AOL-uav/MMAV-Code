function wing = mmav_panel_vehicle_to_vlm_wing(vehicle, component_filter)
%MMAV_PANEL_VEHICLE_TO_VLM_WING Convert MMAV panels into opt_0615 VLM wing struct.
%
% wing = mmav_panel_vehicle_to_vlm_wing(vehicle)
% wing = mmav_panel_vehicle_to_vlm_wing(vehicle, component_filter)
%
% component_filter can be empty, a char/string, or a cell/string array, e.g.
%   'wing'
%   {'wing','curled_tail'}
%
% The opt_0615 solver expects:
%   wing.surfaces{k}.nodes    (Ny+1)x(Nx+1)x3
%   wing.surfaces{k}.panels   struct array with p1,p2,p3,p4,i,j
%
% The current MMAV panels already live in body frame B [m], where x_B is aft.
% No frame conversion is done here.

if nargin < 2
    component_filter = [];
end

panels = vehicle.panels;
if isempty(panels)
    wing = struct('surfaces',{{}});
    return;
end

if ~isempty(component_filter)
    filter = string(component_filter);
    keep = false(numel(panels),1);
    comps = string({panels.component}).';
    for k = 1:numel(filter)
        keep = keep | strcmpi(comps, filter(k));
    end
    panels = panels(keep);
end

% Desired stable order keeps main wing first so old solver reference MAC
% estimates are based on the wing instead of the tail.
preferred = ["wing_R", "wing_L", "curled_tail"];
all_surfs = unique(string({panels.surface}), 'stable');
ordered = strings(0,1);
for k = 1:numel(preferred)
    if any(strcmp(all_surfs, preferred(k)))
        ordered(end+1,1) = preferred(k); %#ok<AGROW>
    end
end
for k = 1:numel(all_surfs)
    if ~any(strcmp(ordered, all_surfs(k)))
        ordered(end+1,1) = all_surfs(k); %#ok<AGROW>
    end
end

surfaces = cell(1, numel(ordered));
for sidx = 1:numel(ordered)
    sname = ordered(sidx);
    idx = strcmp(string({panels.surface}), sname);
    ps = panels(idx);
    if isempty(ps)
        continue;
    end

    Ny = max([ps.i_span]);
    Nx = max([ps.j_chord]);
    nodes = NaN(Ny+1, Nx+1, 3);

    vlm_panels = repmat(base_vlm_panel_struct(), Ny*Nx, 1);
    kk = 0;
    for k = 1:numel(ps)
        P = ps(k);
        i = P.i_span;
        j = P.j_chord;
        nodes(i,   j,   :) = reshape(P.p1_B_m,1,1,3);
        nodes(i+1, j,   :) = reshape(P.p2_B_m,1,1,3);
        nodes(i+1, j+1, :) = reshape(P.p3_B_m,1,1,3);
        nodes(i,   j+1, :) = reshape(P.p4_B_m,1,1,3);

        kk = kk + 1;
        vlm_panels(kk).p1 = P.p1_B_m;
        vlm_panels(kk).p2 = P.p2_B_m;
        vlm_panels(kk).p3 = P.p3_B_m;
        vlm_panels(kk).p4 = P.p4_B_m;
        vlm_panels(kk).i  = i;
        vlm_panels(kk).j  = j;
        vlm_panels(kk).component = P.component;
        vlm_panels(kk).surface = P.surface;
    end
    vlm_panels = vlm_panels(1:kk);

    if any(~isfinite(nodes(:)))
        warning('Surface %s has incomplete node reconstruction. Check panel indexing.', sname);
    end

    S = struct();
    S.name = char(sname);
    S.component = ps(1).component;
    S.nodes = nodes;
    S.panels = vlm_panels;
    surfaces{sidx} = S;
end

wing = struct();
wing.surfaces = surfaces(~cellfun(@isempty, surfaces));
wing.source = 'mmav coarse component panel geometry';
end

% =====================================================================
function p = base_vlm_panel_struct()
p = struct();
p.p1 = [NaN NaN NaN];
p.p2 = [NaN NaN NaN];
p.p3 = [NaN NaN NaN];
p.p4 = [NaN NaN NaN];
p.i = NaN;
p.j = NaN;
p.component = '';
p.surface = '';
end
