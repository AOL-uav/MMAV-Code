function [cl, cd, cm] = xfoil_polar_lookup(polar, alpha_deg, Re, varargin)
%XFOIL_POLAR_LOOKUP  Interpolate (and optionally extrapolate) XFOIL polars.
%
% Supports two polar container types:
%   1) Single polar struct (from xfoil_polar_load)
%   2) Polar family struct  (from xfoil_polar_family_load)
%
% Behavior for a *single* polar matches the original demo:
%   - Reynolds number is clamped to [Re_min, Re_max]
%   - Alpha is interpolated within [alpha_min, alpha_max]
%   - Outside that alpha range, CL/CD are extrapolated using a Viterna-style
%     post-stall extension anchored at the nearest bound.
%   - Cm is held constant outside the data range (not well-defined post-stall)
%
% Inputs
%   polar     : polar struct or polar family struct
%   alpha_deg : query angle of attack [deg]
%   Re        : query Reynolds number [-]
%
% Optional
%   camber_param : scalar parameter for a polar family (e.g., flap deflection)
%
% Outputs
%   cl, cd, cm : section coefficients

% Optional camber/deflection parameter:
%   - ignored for a single polar
%   - required (or defaulted) for a polar *family*
camber_param = [];
if ~isempty(varargin)
    camber_param = varargin{1};
end

% If 'polar' is a family, interpolate across the family parameter first.
if isstruct(polar) && isfield(polar,'type') && strcmpi(polar.type,'family')
    if isempty(camber_param)
        if isfield(polar,'param_default') && ~isempty(polar.param_default)
            camber_param = polar.param_default;
        else
            camber_param = polar.param_list(1);
        end
    end
    [cl, cd, cm] = xfoil_polar_lookup_family(polar, alpha_deg, Re, camber_param);
    return;
end

% ---------------------------------------------------------------------
% Single-polar behavior (original)
% ---------------------------------------------------------------------

% Clamp Re (conservative)
r  = min(max(Re, polar.Re_min), polar.Re_max);
lr = log10(r);

if alpha_deg >= polar.alpha_min && alpha_deg <= polar.alpha_max
    cl = polar.F_CL(alpha_deg, lr);
    cd = polar.F_CD(alpha_deg, lr);
    cm = polar.F_CM(alpha_deg, lr);
    cd = max(cd, 0);
    return;
end

% Anchor at nearest polar bound in alpha
if alpha_deg > polar.alpha_max
    a_s = polar.alpha_max;
else
    a_s = polar.alpha_min;
end

cl_s = polar.F_CL(a_s, lr);
cd_s = polar.F_CD(a_s, lr);
cm_s = polar.F_CM(a_s, lr);

[cl, cd] = viterna_extrap_local(alpha_deg, a_s, cl_s, cd_s, polar.CD90);

% Cm is not reliably defined post-stall; hold constant at the bound.
cm = cm_s;

cd = max(cd, 0);

end

% ---------------------------------------------------------------------
% Local helper: Viterna-style post-stall extrapolation
% ---------------------------------------------------------------------
function [cl, cd] = viterna_extrap_local(alpha_deg, alpha_s_deg, cl_s, cd_s, CD90)
%VITERNA_EXTRAP_LOCAL  Post-stall lift/drag extrapolation (Viterna-style)
%
% Extends a polar beyond its available alpha range while:
%   - matching CL/CD exactly at the anchor alpha_s
%   - tending toward CL -> 0 and CD -> CD90 as |alpha| -> 90 deg

% Avoid singularities near +/-90 deg and sin(alpha)=0
a  = deg2rad(max(min(alpha_deg,  89.9), -89.9));
as = deg2rad(max(min(alpha_s_deg, 89.0), -89.0));

% Coefficients (general form; valid for negative as as well)
A1 = CD90/2;
B1 = CD90;

sinas = sin(as);
cosas = cos(as);
cosas2 = cosas*cosas;

% protect against division by ~0
if abs(sinas) < 1e-6
    sinas = sign(sinas + 1e-12) * 1e-6;
end
if cosas2 < 1e-12
    cosas2 = 1e-12;
end
if abs(cosas) < 1e-6
    cosas = sign(cosas + 1e-12) * 1e-6;
end

% Fit constants to match anchor
% (Form derived from Viterna method; robust for +/- alpha)
K1 = (cd_s - B1*sinas^2) / max(cosas2, 1e-12);
K2 = (cl_s - A1*sin(2*as)) * sinas / max(cosas2, 1e-12);

% Viterna extrapolation
cd = B1*sin(a)^2 + K1*cos(a)^2;
sina = sin(a);
sina_safe = sign(sina + 1e-12) * max(abs(sina), 1e-6);
cl = A1*sin(2*a) + K2*(cos(a)^2) / sina_safe;

end
