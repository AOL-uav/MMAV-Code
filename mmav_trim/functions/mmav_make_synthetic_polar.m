function polar = mmav_make_synthetic_polar(opt)
%MMAV_MAKE_SYNTHETIC_POLAR Build fallback polar struct compatible with xfoil lookup.
%
% This is only a backup when the real XFOIL polar data file is missing.  It
% returns the same fields used by xfoil_polar_lookup.m.

if nargin < 1 || isempty(opt), opt = struct(); end
alpha0_deg = getfield_default(opt,'alpha0_deg',-2.0);
CLa        = getfield_default(opt,'CLa_per_rad',5.7);
CL_min     = getfield_default(opt,'CL_min',-1.25);
CL_max     = getfield_default(opt,'CL_max', 1.35);
CD0        = getfield_default(opt,'CD0',0.035);
k          = getfield_default(opt,'k',0.080);
Cm0        = getfield_default(opt,'Cm0',-0.045);

alpha_vec = (-25:1:25).';
Re_vec = [8000 12000 20000 35000 50000 75000 100000].';

AA = [];
RR = [];
CL = [];
CD = [];
CM = [];
AF = strings(0,1);
for i = 1:numel(Re_vec)
    Re = Re_vec(i);
    Re_factor = min(max((log10(Re)-log10(8000))/(log10(100000)-log10(8000)),0),1);
    for j = 1:numel(alpha_vec)
        a = alpha_vec(j);
        cl_lin = CLa * deg2rad(a - alpha0_deg);
        cl = min(max(cl_lin, CL_min), CL_max);
        % low-Re penalty in drag, intentionally conservative
        cd0_eff = CD0 * (1.25 - 0.25*Re_factor);
        cd = cd0_eff + k*cl^2;
        cm = Cm0;
        AA(end+1,1) = a; %#ok<AGROW>
        RR(end+1,1) = Re; %#ok<AGROW>
        CL(end+1,1) = cl; %#ok<AGROW>
        CD(end+1,1) = cd; %#ok<AGROW>
        CM(end+1,1) = cm; %#ok<AGROW>
        AF(end+1,1) = "synthetic"; %#ok<AGROW>
    end
end

logRe = log10(RR);
polar = struct();
polar.airfoil = "synthetic_fallback";
polar.alpha_min = min(AA);
polar.alpha_max = max(AA);
polar.Re_min    = min(RR);
polar.Re_max    = max(RR);
polar.CD90      = 1.0;
polar.raw = struct('airfoil',AF,'Re',RR,'alpha',AA,'CL',CL,'CD',CD,'CM',CM,'logRe',logRe);
polar.F_CL = scatteredInterpolant(AA, logRe, CL, 'linear', 'nearest');
polar.F_CD = scatteredInterpolant(AA, logRe, CD, 'linear', 'nearest');
polar.F_CM = scatteredInterpolant(AA, logRe, CM, 'linear', 'nearest');
polar.is_synthetic = true;
end

% =====================================================================
function v = getfield_default(s, field, default)
if isfield(s, field)
    v = s.(field);
else
    v = default;
end
end
