function vlm_cfg = mmav_make_vlm_config(preferred_polar)
%MMAV_MAKE_VLM_CONFIG Create a VLM adapter config with stable polar search.
%
% preferred_polar may be empty, a filename on the MATLAB path, or a full path.
% For final runs, pass the exact XFOIL polar file, e.g.
%   vlm_cfg = mmav_make_vlm_config(fullfile('path','NACA_0008.dat'));

if nargin < 1
    preferred_polar = '';
end

vlm_cfg = mmav_default_vlm_config();
vlm_cfg.verbose = true;
vlm_cfg.compute_separate_component_diagnostics = false;

% Stable search order for this project.  Explicit preferred_polar still wins.
vlm_cfg.polar_search_files = { ...
    'NACA_0008.dat', 'NACA0008.dat', 'naca0008.dat', ...
    'NACA_2408.dat', 'NACA2408.dat', 'naca2408.dat'};

if ~isempty(preferred_polar)
    vlm_cfg.polar_file = preferred_polar;
end
end
