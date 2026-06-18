function [cl, cd, cm] = xfoil_polar_lookup_family(family, alpha_deg, Re, param)
%XFOIL_POLAR_LOOKUP_FAMILY  Interpolate a polar family across a camber/flap parameter.
%
% family must be created by xfoil_polar_family_load() or follow the same structure:
%   family.type = 'family'
%   family.param_list (N x 1)
%   family.polars {N}
%
% Interpolates CL/CD/CM linearly in 'param' between bracketing polars.
% Outside the param range, clamps to the nearest polar.

if ~isstruct(family) || ~isfield(family,'param_list') || ~isfield(family,'polars')
    error('Invalid polar family struct.');
end

p = family.param_list(:);
N = numel(p);

if N < 1 || numel(family.polars) ~= N
    error('Polar family param_list/polars size mismatch.');
end

% Clamp outside the range
if param <= p(1)
    [cl, cd, cm] = xfoil_polar_lookup(family.polars{1}, alpha_deg, Re);
    return;
elseif param >= p(end)
    [cl, cd, cm] = xfoil_polar_lookup(family.polars{end}, alpha_deg, Re);
    return;
end

% Find bracket
i2 = find(p >= param, 1, 'first');
i1 = i2 - 1;

p1 = p(i1);
p2 = p(i2);

if abs(p2 - p1) < 1e-12
    [cl, cd, cm] = xfoil_polar_lookup(family.polars{i1}, alpha_deg, Re);
    return;
end

w = (param - p1) / (p2 - p1);

[cl1, cd1, cm1] = xfoil_polar_lookup(family.polars{i1}, alpha_deg, Re);
[cl2, cd2, cm2] = xfoil_polar_lookup(family.polars{i2}, alpha_deg, Re);

cl = (1-w)*cl1 + w*cl2;
cd = (1-w)*cd1 + w*cd2;
cm = (1-w)*cm1 + w*cm2;

cd = max(cd, 0);

end
