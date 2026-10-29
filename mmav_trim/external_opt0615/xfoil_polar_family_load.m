function family = xfoil_polar_family_load(file_list, param_list, param_name)
%XFOIL_POLAR_FAMILY_LOAD  Load a family of XFOIL polars parameterized by camber/flap.
%
% This is a lightweight container that lets the solver interpolate across an
% additional 1D parameter (e.g., flap deflection or camber setting).
%
% Inputs
%   file_list  : cell array of polar filenames OR string array
%   param_list : numeric vector same length as file_list (e.g., flap deg)
%   param_name : optional string (default: 'camber_param')
%
% Output
%   family : struct with fields
%       .type         = 'family'
%       .param_name   : name of the parameter
%       .param_list   : sorted list of parameter values
%       .polars       : cell array of polar structs (same order as param_list)
%       .param_default: default parameter (closest to 0 if present)
%
% Usage
%   files = {'polar_flap_-5.dat','polar_flap_0.dat','polar_flap_+5.dat'};
%   deltas = [-5, 0, 5];
%   polar = xfoil_polar_family_load(files, deltas, 'flap_deg');
%
% Then in the solver:
%   [cl,cd,cm] = xfoil_polar_lookup(polar, alpha_deg, Re, flap_deg);

if nargin < 3 || isempty(param_name)
    param_name = 'camber_param';
end

% Normalize inputs
if isstring(file_list)
    file_list = cellstr(file_list);
elseif ischar(file_list)
    file_list = {file_list};
end

param_list = param_list(:);

if numel(file_list) ~= numel(param_list)
    error('file_list and param_list must have the same length.');
end

% Sort by parameter
[param_list, idx] = sort(param_list);
file_list = file_list(idx);

N = numel(file_list);
polars = cell(N,1);

for k = 1:N
    polars{k} = xfoil_polar_load(file_list{k});
end

family = struct();
family.type       = 'family';
family.param_name = param_name;
family.param_list = param_list;
family.polars     = polars;

% Default: parameter closest to 0 (nice for flap families)
[~, k0] = min(abs(param_list));
family.param_default = param_list(k0);

% Convenience bounds (not strictly required; each polar clamps internally)
family.alpha_min = min(cellfun(@(p)p.alpha_min, polars));
family.alpha_max = max(cellfun(@(p)p.alpha_max, polars));
family.Re_min    = min(cellfun(@(p)p.Re_min,    polars));
family.Re_max    = max(cellfun(@(p)p.Re_max,    polars));

end
