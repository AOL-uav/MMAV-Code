function pm = wingopt_planform_metrics(b, eta_cp, xLE_cp, xTE_cp, interp_method)
%WINGOPT_PLANFORM_METRICS  Geometric metrics for a half-span planform table.
%
% Inputs
%   b           : full span [m]
%   eta_cp      : control-point span fractions in [0,1]
%   xLE_cp/xTE_cp : control-point LE/TE x-locations [m]
%   interp_method: interpolation method for fine resampling
%
% Outputs (pm struct)
%   .S_half, .S_total, .AR
%   .MAC
%   .xLE_MAC        : area-weighted leading-edge x for the equivalent MAC
%   .xQC_MAC        : area-weighted quarter-chord x
%   .y_area         : area-weighted span station on the half-wing
%   .c_root, .c_tip, .c_min, .c_max, .taper_ratio
%   .eta_fine, .y_fine, .c_fine, .xLE_fine, .xTE_fine

if nargin < 5 || isempty(interp_method)
    interp_method = 'pchip';
end

eta_cp = eta_cp(:);
xLE_cp = xLE_cp(:);
xTE_cp = xTE_cp(:);

if numel(eta_cp) ~= numel(xLE_cp) || numel(eta_cp) ~= numel(xTE_cp)
    error('wingopt_planform_metrics:SizeMismatch', ...
        'eta_cp, xLE_cp, and xTE_cp must have the same length.');
end

eta_fine = linspace(0, 1, 2001).';
halfspan = 0.5 * b;
y_fine = eta_fine * halfspan;

xLE_fine = interp1(eta_cp, xLE_cp, eta_fine, interp_method, 'extrap');
xTE_fine = interp1(eta_cp, xTE_cp, eta_fine, interp_method, 'extrap');
c_fine = max(xTE_fine - xLE_fine, 1e-9);

S_half = trapz(y_fine, c_fine);
S_total = 2 * S_half;

if S_half <= 1e-12
    MAC = NaN;
    xLE_MAC = NaN;
    xQC_MAC = NaN;
    y_area = NaN;
else
    MAC = trapz(y_fine, c_fine.^2) / S_half;
    xLE_MAC = trapz(y_fine, xLE_fine .* c_fine) / S_half;
    xQC_MAC = trapz(y_fine, (xLE_fine + 0.25 * c_fine) .* c_fine) / S_half;
    y_area = trapz(y_fine, y_fine .* c_fine) / S_half;
end

pm = struct();
pm.S_half = S_half;
pm.S_total = S_total;
pm.AR = b^2 / max(S_total, 1e-12);
pm.MAC = MAC;
pm.xLE_MAC = xLE_MAC;
pm.xQC_MAC = xQC_MAC;
pm.y_area = y_area;
pm.c_root = c_fine(1);
pm.c_tip = c_fine(end);
pm.c_min = min(c_fine);
pm.c_max = max(c_fine);
pm.taper_ratio = c_fine(end) / max(c_fine(1), 1e-12);
pm.eta_fine = eta_fine;
pm.y_fine = y_fine;
pm.c_fine = c_fine;
pm.xLE_fine = xLE_fine;
pm.xTE_fine = xTE_fine;

end
