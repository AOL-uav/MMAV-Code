function drag_panels = mmav_assign_tail_drag_loads(tail_panels, state, tail_cfg)
%MMAV_ASSIGN_TAIL_DRAG_LOADS Assign drag-only loads to curled-tail panels.
%
% The fuselage-attached curled tail is treated as a drag-producing body/sheet,
% not a lifting surface.  Each panel receives a force along the relative-wind
% drag direction:
%
%   F_p^B = q * CD * A_projected * e_D^B
%
% where e_D^B is the drag-force direction in the body/build frame and
%
%   A_projected = A_panel * | n_panel^B dot e_D^B |
%
% for the default 'projected_normal' model.
%
% No lift coefficient and no VLM circulation are used for these tail panels.

if nargin < 3 || isempty(tail_cfg)
    tail_cfg = struct();
end
if isempty(tail_panels)
    drag_panels = tail_panels;
    return;
end

rho = getfield_default(state, 'rho_kgpm3', 1.225);
V   = getfield_default(state, 'V_mps', 5.8);
q   = 0.5*rho*V^2;
axesB = mmav_wind_axes_B(state);
eD = axesB.eD_B;

CD = getfield_default(tail_cfg, 'drag_CD', 1.30);
area_model = char(string(getfield_default(tail_cfg, 'drag_area_model', 'projected_normal')));
scale = getfield_default(tail_cfg, 'drag_scale', 1.0);

drag_panels = tail_panels;
for k = 1:numel(drag_panels)
    n = drag_panels(k).normal_B;
    n = n / max(norm(n), 1e-12);
    A = drag_panels(k).area_m2;

    switch lower(area_model)
        case {'projected_normal','normal_projection','projected'}
            Aproj = A * abs(dot(n, eD));
        case {'full_area','area'}
            Aproj = A;
        otherwise
            error('Unknown tail drag_area_model: %s', area_model);
    end

    D = q * CD * Aproj * scale;
    drag_panels(k).component = 'curled_tail_drag';
    drag_panels(k).surface = 'curled_tail_drag';
    drag_panels(k).F_B_N = D * eD;
    drag_panels(k).M_ref_B_Nm = [0 0 0];
    drag_panels(k).CD_drag_used = CD;
    drag_panels(k).A_projected_m2 = Aproj;
end
end

% =====================================================================
function v = getfield_default(s, field, default)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end
