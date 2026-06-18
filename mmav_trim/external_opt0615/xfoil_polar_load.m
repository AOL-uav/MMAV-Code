function polar = xfoil_polar_load(fname)
%XFOIL_POLAR_LOAD  Load an XFOIL polar table (alpha/Re -> Cl/Cd/Cm).
%
% Parses the typical XFOIL polar text format, e.g.:
%   Airfoil Re Ncrit Iter alpha CL CD CDp CM Top_Xtr Bot_Xtr Note
%
% The returned 'polar' is a struct containing:
%   - alpha_min/max, Re_min/max
%   - scatteredInterpolant objects in (alpha, log10(Re)) space for CL/CD/CM
%   - CD90 : post-stall drag constant used for Viterna-style extrapolation
%
% Notes on flagged points:
% Some datasets include a 'Note' field with flags like FLAG(CL), FLAG(MONO), etc.
% Here we drop only FLAG(CL) points, which are typically true convergence outliers.

if exist(fname,'file') ~= 2
    error('Polar file not found: %s', fname);
end

fid = fopen(fname,'r');
if fid < 0
    error('Failed to open polar file: %s', fname);
end

% first non-empty line is header
hdr = '';
while ischar(hdr)
    hdr = fgetl(fid);
    if ~ischar(hdr)
        break;
    end
    if ~isempty(strtrim(hdr))
        break;
    end
end

if ~ischar(hdr)
    fclose(fid);
    error('Polar file appears empty: %s', fname);
end

% Data line format (based on provided .dat examples):
% Airfoil(str) Re Ncrit Iter alpha CL CD CDp CM Top_Xtr Bot_Xtr Note(str)
C = textscan(fid, '%s %f %f %f %f %f %f %f %f %f %f %s', ...
    'Delimiter',' ', 'MultipleDelimsAsOne',true, 'ReturnOnError',false);

fclose(fid);

airfoil = string(C{1});
Re      = C{2};
alpha   = C{5};
CL      = C{6};
CD      = C{7};
CM      = C{9};
note    = string(C{12});

% drop NaNs
ok = isfinite(Re) & isfinite(alpha) & isfinite(CL) & isfinite(CD) & isfinite(CM);

% remove obvious CL outliers flagged by the dataset (if any)
ok = ok & ~contains(note, "FLAG(CL)");

airfoil = airfoil(ok);
Re      = Re(ok);
alpha   = alpha(ok);
CL      = CL(ok);
CD      = CD(ok);
CM      = CM(ok);

% Build interpolants in (alpha, log10(Re)) space
logRe = log10(Re);

polar = struct();
polar.airfoil = unique(airfoil);
polar.alpha_min = min(alpha);
polar.alpha_max = max(alpha);
polar.Re_min    = min(Re);
polar.Re_max    = max(Re);

% Store filtered raw samples for validation/plotting.
% This is a non-breaking addition (the solver uses only the interpolants).
polar.raw = struct();
polar.raw.airfoil = airfoil;
polar.raw.Re      = Re;
polar.raw.alpha   = alpha;
polar.raw.CL      = CL;
polar.raw.CD      = CD;
polar.raw.CM      = CM;
polar.raw.logRe   = logRe;

% Post-stall extrapolation setting (used when alpha exceeds data range)
polar.CD90 = 1.0; % ~2D flat-plate Cd at 90 deg (engineering default)

% Inside convex hull: linear; outside: nearest
polar.F_CL = scatteredInterpolant(alpha, logRe, CL, 'linear', 'nearest');
polar.F_CD = scatteredInterpolant(alpha, logRe, CD, 'linear', 'nearest');
polar.F_CM = scatteredInterpolant(alpha, logRe, CM, 'linear', 'nearest');

end
