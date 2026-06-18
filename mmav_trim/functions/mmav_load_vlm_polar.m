function [polar, info] = mmav_load_vlm_polar(vlm_cfg)
%MMAV_LOAD_VLM_POLAR Load a real XFOIL polar or fallback synthetic polar.
%
% [polar, info] = mmav_load_vlm_polar(vlm_cfg)
%
% Search order:
%   1. vlm_cfg.polar_file if provided
%   2. common filenames from vlm_cfg.polar_search_files on MATLAB path/current dir
%   3. synthetic fallback if allowed

if nargin < 1 || isempty(vlm_cfg)
    vlm_cfg = mmav_default_vlm_config();
end

info = struct('source','', 'path','', 'is_synthetic',false, 'message','');

% Explicit file.
if isfield(vlm_cfg,'polar_file') && ~isempty(vlm_cfg.polar_file)
    f = char(vlm_cfg.polar_file);
    if exist(f,'file') == 2
        polar = xfoil_polar_load(f);
        info.source = 'file';
        info.path = which_or_self(f);
        info.is_synthetic = false;
        return;
    else
        warning('Requested polar file not found: %s', f);
    end
end

% Auto-search common filenames.
search_files = {};
if isfield(vlm_cfg,'polar_search_files') && ~isempty(vlm_cfg.polar_search_files)
    search_files = vlm_cfg.polar_search_files;
end
if ischar(search_files) || isstring(search_files)
    search_files = cellstr(search_files);
end

for k = 1:numel(search_files)
    f = char(search_files{k});
    if exist(f,'file') == 2
        polar = xfoil_polar_load(f);
        info.source = 'auto_file';
        info.path = which_or_self(f);
        info.is_synthetic = false;
        return;
    end
end

% Fallback synthetic polar.
allow_fallback = getfield_default(vlm_cfg,'allow_synthetic_polar_if_missing',true);
if allow_fallback
    opt = getfield_default(vlm_cfg,'synthetic_polar',struct());
    polar = mmav_make_synthetic_polar(opt);
    info.source = 'synthetic';
    info.path = '';
    info.is_synthetic = true;
    info.message = 'No real polar file found; using synthetic fallback polar.';
    if getfield_default(vlm_cfg,'verbose',true)
        warning('%s Set vlm_cfg.polar_file to the real XFOIL polar for final use.', info.message);
    end
    return;
end

error('No polar file found and synthetic fallback is disabled. Set vlm_cfg.polar_file.');
end

% =====================================================================
function p = which_or_self(f)
w = which(f);
if isempty(w)
    p = f;
else
    p = w;
end
end

% =====================================================================
function v = getfield_default(s, field, default)
if isfield(s, field)
    v = s.(field);
else
    v = default;
end
end
