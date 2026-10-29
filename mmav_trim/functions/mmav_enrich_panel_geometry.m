function p = mmav_enrich_panel_geometry(p, preferred_normal_B)
%MMAV_ENRICH_PANEL_GEOMETRY Add centroid, area, normal, chord/span axes.
%
% p must contain p1_B_m ... p4_B_m as row vectors.
% preferred_normal_B is optional. If provided, panel normal is flipped so
% dot(normal, preferred_normal_B) >= 0 when possible.

if nargin < 2
    preferred_normal_B = [];
end

p1 = p.p1_B_m; p2 = p.p2_B_m; p3 = p.p3_B_m; p4 = p.p4_B_m;

p.centroid_B_m = 0.25 * (p1 + p2 + p3 + p4);

% Area from two triangles.
A1_vec = cross(p2 - p1, p3 - p1);
A2_vec = cross(p3 - p1, p4 - p1);
p.area_m2 = 0.5*norm(A1_vec) + 0.5*norm(A2_vec);

% Normal from corner ordering. Use p4-p1 x p2-p1 to make right-wing
% trapezoid panels point upward before optional flipping.
n = cross(p4 - p1, p2 - p1);
if norm(n) < 1e-14
    n = A1_vec + A2_vec;
end
if norm(n) < 1e-14
    p.normal_B = [0 0 1];
else
    n = n / norm(n);
    if ~isempty(preferred_normal_B)
        pref = preferred_normal_B(:).';
        if norm(pref) > 1e-14
            pref = pref / norm(pref);
            if dot(n, pref) < 0
                n = -n;
            end
        end
    end
    p.normal_B = n;
end

% Approximate chord and span directions at panel center.
le_mid = 0.5 * (p1 + p2);
te_mid = 0.5 * (p4 + p3);
ch = te_mid - le_mid;
if norm(ch) < 1e-14
    p.chord_hat_B = [1 0 0];
else
    p.chord_hat_B = ch / norm(ch);
end

inboard_mid = 0.5 * (p1 + p4);
outboard_mid = 0.5 * (p2 + p3);
sp = outboard_mid - inboard_mid;
if norm(sp) < 1e-14
    p.span_hat_B = [0 1 0];
else
    p.span_hat_B = sp / norm(sp);
end
end
