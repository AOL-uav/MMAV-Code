function cfg = mmav_set_tail_design(cfg, tail)
%MMAV_SET_TAIL_DESIGN Set physical fuselage-attached curled-tail geometry.
%
% This helper is intentionally dimension-based rather than optimizer-style.
% Useful fields in tail:
%   enabled          true/false
%   origin_B_mm      [x y z] leading-edge/attachment center in body frame
%   span_mm          total span in y_B
%   arc_chord_mm     arc length along curled sheet
%   curl_deg         total curl angle; positive curls upward into +z_B
%   incidence_deg    initial tangent incidence angle
%   Nx, Ny           panel counts
%   enforce_gap      if true, clips span to fit inside exposed-wing gap
%   gap_clearance_mm side clearance to keep from exposed wing panels
%   aero_mode        'drag_only' or 'vlm_lifting_surface'
%   drag_CD          drag-only tail coefficient
%   drag_area_model  projected-area model for drag-only tail
%
% Diagnostics are stored in cfg.curled_tail.* for later tables.

if nargin < 2 || isempty(tail)
    return;
end

base = cfg.curled_tail;

if isfield(tail,'enabled') && ~tail.enabled
    cfg.curled_tail.enabled = false;
    cfg.curled_tail.span_mm = 0;
    cfg.curled_tail.arc_chord_mm = 0;
    return;
end

cfg.curled_tail.enabled = true;

fields = {'origin_B_mm','span_mm','arc_chord_mm','curl_deg','incidence_deg','Nx','Ny', ...
          'aero_mode','drag_CD','drag_area_model','drag_scale'};
for k = 1:numel(fields)
    f = fields{k};
    if isfield(tail,f) && ~isempty(tail.(f))
        cfg.curled_tail.(f) = tail.(f);
    end
end

if isfield(tail,'enforce_gap') && tail.enforce_gap
    clearance = getfield_default(tail, 'gap_clearance_mm', 3.0);
    max_span = max(0, 2*(cfg.wing.y_attach_mm - clearance));
    if cfg.curled_tail.span_mm > max_span
        cfg.curled_tail.span_mm = max_span;
    end
    cfg.curled_tail.enforced_gap_clearance_mm = clearance;
end

% Area diagnostics relative to the original baseline tail in cfg.
orig_area = max(base.span_mm * base.arc_chord_mm, 1e-12);
cfg.curled_tail.area_mm2 = cfg.curled_tail.span_mm * cfg.curled_tail.arc_chord_mm;
cfg.curled_tail.actual_area_scale_relative_to_original = cfg.curled_tail.area_mm2 / orig_area;
cfg.curled_tail.linear_scale_equivalent = sqrt(max(cfg.curled_tail.actual_area_scale_relative_to_original,0));

% Consistent short note for printouts.
mode = getfield_default(cfg.curled_tail, 'aero_mode', 'drag_only');
cfg.curled_tail.design_note = sprintf('mode %s, span %.1f mm, arc %.1f mm, curl %.1f deg, inc %.1f deg', ...
    char(string(mode)), cfg.curled_tail.span_mm, cfg.curled_tail.arc_chord_mm, ...
    cfg.curled_tail.curl_deg, cfg.curled_tail.incidence_deg);
end

function v = getfield_default(s, field, default)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end
