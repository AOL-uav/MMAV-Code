function axesB = mmav_wind_axes_B(state)
%MMAV_WIND_AXES_B Return drag/lift axes in the x-aft, z-up body frame.
%
% eD_B is the relative-wind / drag-force direction. With x_B aft, alpha=0
% gives eD_B = [1 0 0]. A positive alpha tilts eD_B toward +z_B.
%
% eL_B is approximately upward lift direction for small positive alpha.

alpha = deg2rad(getfield_default(state, 'alpha_deg', 0));
beta  = deg2rad(getfield_default(state, 'beta_deg', 0));

eD = [cos(alpha)*cos(beta), cos(alpha)*sin(beta), sin(alpha)];
eD = eD / max(norm(eD), 1e-12);

zB = [0 0 1];
eY = cross(zB, eD);
if norm(eY) < 1e-12
    eY = [0 1 0];
else
    eY = eY / norm(eY);
end

eL = cross(eD, eY);
eL = eL / max(norm(eL), 1e-12);

axesB = struct('eD_B',eD, 'eY_B',eY, 'eL_B',eL);
end

% =====================================================================
function v = getfield_default(s, field, default)
if isfield(s, field)
    v = s.(field);
else
    v = default;
end
end
