function info = mmav_print_vlm_polar_info(vlm_cfg)
%MMAV_PRINT_VLM_POLAR_INFO Load once and print the polar source.
[~, info] = mmav_load_vlm_polar(vlm_cfg);
fprintf('\n--- VLM polar source ---\n');
fprintf('  source       : %s\n', info.source);
if isfield(info,'path') && ~isempty(info.path)
    fprintf('  path         : %s\n', info.path);
end
if isfield(info,'is_synthetic') && info.is_synthetic
    fprintf('  WARNING      : synthetic fallback polar is active.\n');
end
end
