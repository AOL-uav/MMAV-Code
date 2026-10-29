function [r_cg_B_m, total_mass_kg, T] = mmav_compute_total_cg(mass_components)
%MMAV_COMPUTE_TOTAL_CG Compute total CG from coarse mass components.
%
% mass_components(k):
%   .name
%   .mass_kg
%   .r_cg_B_m = [x y z] in body frame B

if isempty(mass_components)
    error('No mass components were provided.');
end

m = zeros(numel(mass_components),1);
r = zeros(numel(mass_components),3);
name = strings(numel(mass_components),1);

for k = 1:numel(mass_components)
    name(k) = string(mass_components(k).name);
    m(k) = mass_components(k).mass_kg;
    r(k,:) = mass_components(k).r_cg_B_m;
end

active = isfinite(m) & (m > 0);
total_mass_kg = sum(m(active));
if total_mass_kg <= 0
    error('Total mass must be positive.');
end

r_cg_B_m = sum(r(active,:) .* m(active), 1) / total_mass_kg;

T = table(name, 1e3*m, 1e3*r(:,1), 1e3*r(:,2), 1e3*r(:,3), ...
    'VariableNames', {'name','mass_g','x_cg_B_mm','y_cg_B_mm','z_cg_B_mm'});
end
