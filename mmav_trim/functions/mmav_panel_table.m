function T = mmav_panel_table(panels, max_rows)
%MMAV_PANEL_TABLE Create a compact table of panel centroid/area/normal data.

if nargin < 2 || isempty(max_rows)
    max_rows = numel(panels);
end
n = min(numel(panels), max_rows);

id = zeros(n,1);
component = strings(n,1);
surface = strings(n,1);
i_span = zeros(n,1);
j_chord = zeros(n,1);
x_centroid_B_mm = zeros(n,1);
y_centroid_B_mm = zeros(n,1);
z_centroid_B_mm = zeros(n,1);
area_m2 = zeros(n,1);
nx_B = zeros(n,1);
ny_B = zeros(n,1);
nz_B = zeros(n,1);

for k = 1:n
    id(k) = panels(k).id;
    component(k) = string(panels(k).component);
    surface(k) = string(panels(k).surface);
    i_span(k) = panels(k).i_span;
    j_chord(k) = panels(k).j_chord;
    x_centroid_B_mm(k) = 1e3*panels(k).centroid_B_m(1);
    y_centroid_B_mm(k) = 1e3*panels(k).centroid_B_m(2);
    z_centroid_B_mm(k) = 1e3*panels(k).centroid_B_m(3);
    area_m2(k) = panels(k).area_m2;
    nx_B(k) = panels(k).normal_B(1);
    ny_B(k) = panels(k).normal_B(2);
    nz_B(k) = panels(k).normal_B(3);
end

T = table(id, component, surface, i_span, j_chord, ...
    x_centroid_B_mm, y_centroid_B_mm, z_centroid_B_mm, ...
    area_m2, nx_B, ny_B, nz_B);
end
