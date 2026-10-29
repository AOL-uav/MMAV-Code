function panels = mmav_make_curled_tail_panels(t)
%MMAV_MAKE_CURLED_TAIL_PANELS Build a centerline curled-tail panel surface.
%
% The tail is represented as a rectangular sheet whose chord direction curls
% upward in the x-z plane. This gives panel centroids/moment arms for later
% panel-method loads.
%
% Body frame B:
%   x_B aft, y_B right, z_B up
%
% t.origin_B_mm is the leading-edge / attachment-center point.
% t.arc_chord_mm is arc length, not projected chord.
% t.curl_deg > 0 curls upward into +z_B.

if ~t.enabled
    panels = struct([]);
    return;
end

Nx = max(1, round(t.Nx));
Ny = max(1, round(t.Ny));

s_nodes = linspace(0, t.arc_chord_mm, Nx+1);       % chord/arc coordinate [mm]
y_nodes = linspace(-0.5*t.span_mm, 0.5*t.span_mm, Ny+1);

x_arc = zeros(size(s_nodes));
z_arc = zeros(size(s_nodes));

inc = deg2rad(t.incidence_deg);
curl = deg2rad(t.curl_deg);
arc_len = max(t.arc_chord_mm, 1e-12);
kappa = curl / arc_len;    % signed curvature [rad/mm]

for j = 1:numel(s_nodes)
    s = s_nodes(j);
    if abs(kappa) < 1e-12
        x_arc(j) = s*cos(inc);
        z_arc(j) = s*sin(inc);
    else
        theta = inc + kappa*s;
        x_arc(j) = (sin(theta) - sin(inc)) / kappa;
        z_arc(j) = (-cos(theta) + cos(inc)) / kappa;
    end
end

nodes_B_m = zeros(Ny+1, Nx+1, 3);
for i = 1:Ny+1
    for j = 1:Nx+1
        r_mm = t.origin_B_mm + [x_arc(j), y_nodes(i), z_arc(j)];
        nodes_B_m(i,j,:) = r_mm * 1e-3;
    end
end

panels = repmat(base_panel_struct(), 0, 1);
idx = 0;
for i = 1:Ny
    for j = 1:Nx
        idx = idx + 1;
        p = base_panel_struct();
        p.component = 'curled_tail';
        p.surface = 'curled_tail';
        p.i_span = i;
        p.j_chord = j;
        p.p1_B_m = squeeze(nodes_B_m(i,   j,   :)).';
        p.p2_B_m = squeeze(nodes_B_m(i+1, j,   :)).';
        p.p3_B_m = squeeze(nodes_B_m(i+1, j+1, :)).';
        p.p4_B_m = squeeze(nodes_B_m(i,   j+1, :)).';
        p = mmav_enrich_panel_geometry(p, [0 0 1]);
        panels(idx,1) = p; %#ok<AGROW>
    end
end

end

% =====================================================================
function p = base_panel_struct()
p = struct();
p.id = NaN;
p.component = '';
p.surface = '';
p.i_span = NaN;
p.j_chord = NaN;
p.p1_B_m = [NaN NaN NaN];
p.p2_B_m = [NaN NaN NaN];
p.p3_B_m = [NaN NaN NaN];
p.p4_B_m = [NaN NaN NaN];
p.centroid_B_m = [NaN NaN NaN];
p.area_m2 = NaN;
p.normal_B = [NaN NaN NaN];
p.chord_hat_B = [NaN NaN NaN];
p.span_hat_B = [NaN NaN NaN];
p.F_B_N = [0 0 0];
p.M_ref_B_Nm = [0 0 0];
end
