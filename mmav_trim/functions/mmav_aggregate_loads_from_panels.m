function loads = mmav_aggregate_loads_from_panels(panels, aero_bodies, r_CG_B_m)
%MMAV_AGGREGATE_LOADS_FROM_PANELS Sum panel/body forces and moments about CG.
%
% Core equation:
%   M_CG_B = sum_p [ M_p_ref_B + (r_p_B - r_CG_B) x F_p_B ]
%          + sum_b [ M_b_ref_B + (r_b_B - r_CG_B) x F_b_B ]
%
% panels(k) requires:
%   .centroid_B_m, .F_B_N, optional .M_ref_B_Nm, .component
%
% aero_bodies(k) requires:
%   .r_ref_B_m, .F_B_N, optional .M_ref_B_Nm, .name

if nargin < 2 || isempty(aero_bodies)
    aero_bodies = struct([]);
end
validateattributes(r_CG_B_m, {'numeric'}, {'vector','numel',3,'finite'});
r_CG_B_m = r_CG_B_m(:).';

F_total = [0 0 0];
M_total = [0 0 0];

source_name = strings(0,1);
source_type = strings(0,1);
Fx = []; Fy = []; Fz = [];
Mx = []; My = []; Mz = [];

% Panel loads.
if ~isempty(panels)
    components = unique(string({panels.component}));
    for cidx = 1:numel(components)
        comp = components(cidx);
        idx = strcmp(string({panels.component}), comp);
        F_comp = [0 0 0];
        M_comp = [0 0 0];

        for k = find(idx)
            F = panels(k).F_B_N;
            Mref = getfield_default(panels(k), 'M_ref_B_Nm', [0 0 0]);
            r = panels(k).centroid_B_m - r_CG_B_m;
            M = Mref + cross(r, F);

            F_comp = F_comp + F;
            M_comp = M_comp + M;
        end

        F_total = F_total + F_comp;
        M_total = M_total + M_comp;

        source_name(end+1,1) = comp; %#ok<AGROW>
        source_type(end+1,1) = "panel_surface"; %#ok<AGROW>
        Fx(end+1,1) = F_comp(1); Fy(end+1,1) = F_comp(2); Fz(end+1,1) = F_comp(3); %#ok<AGROW>
        Mx(end+1,1) = M_comp(1); My(end+1,1) = M_comp(2); Mz(end+1,1) = M_comp(3); %#ok<AGROW>
    end
end

% Body loads such as fuselage drag.
for k = 1:numel(aero_bodies)
    if isfield(aero_bodies(k),'enabled') && ~aero_bodies(k).enabled
        continue;
    end
    F = aero_bodies(k).F_B_N;
    Mref = getfield_default(aero_bodies(k), 'M_ref_B_Nm', [0 0 0]);
    r = aero_bodies(k).r_ref_B_m - r_CG_B_m;
    M = Mref + cross(r, F);

    F_total = F_total + F;
    M_total = M_total + M;

    source_name(end+1,1) = string(aero_bodies(k).name); %#ok<AGROW>
    source_type(end+1,1) = "aero_body"; %#ok<AGROW>
    Fx(end+1,1) = F(1); Fy(end+1,1) = F(2); Fz(end+1,1) = F(3); %#ok<AGROW>
    Mx(end+1,1) = M(1); My(end+1,1) = M(2); Mz(end+1,1) = M(3); %#ok<AGROW>
end

loads = struct();
loads.F_B_N = F_total;
loads.M_CG_B_Nm = M_total;
loads.r_CG_B_m = r_CG_B_m;
loads.tables.by_source = table(source_name, source_type, Fx, Fy, Fz, Mx, My, Mz);
end

% =====================================================================
function v = getfield_default(s, field, default)
if isfield(s, field)
    v = s.(field);
else
    v = default;
end
end
