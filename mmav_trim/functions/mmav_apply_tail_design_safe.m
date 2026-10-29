function cfg = mmav_apply_tail_design_safe(base_cfg, area_scale, curl_deg, incidence_deg, clearance_mm)
%MMAV_APPLY_TAIL_DESIGN_SAFE Scale curled tail area without growing into wing panels.
%
% This is intended for the integrated VLM model.  The earlier helper scaled
% span and chord by sqrt(area_scale), which can make the centerline tail
% physically overlap the exposed wing panels.  This helper first clips the
% tail span to fit inside |y| < y_attach-clearance and then applies area
% scaling by changing the arc/chord length only.
%
% area_scale is relative to the *safe baseline* geometry after span clipping.

if nargin < 3 || isempty(curl_deg)
    curl_deg = base_cfg.curled_tail.curl_deg;
end
if nargin < 4 || isempty(incidence_deg)
    incidence_deg = base_cfg.curled_tail.incidence_deg;
end
if nargin < 5 || isempty(clearance_mm)
    clearance_mm = 3.0;
end

cfg = base_cfg;

if area_scale <= 0
    cfg.curled_tail.enabled = false;
    cfg.curled_tail.span_mm = 0;
    cfg.curled_tail.arc_chord_mm = 0;
    cfg.curled_tail.curl_deg = curl_deg;
    cfg.curled_tail.incidence_deg = incidence_deg;
    cfg.curled_tail.scale_note = 'tail disabled';
    return;
end

if ~isfield(cfg,'wing') || ~isfield(cfg.wing,'y_attach_mm')
    error('cfg.wing.y_attach_mm is required for safe tail scaling.');
end

max_span = max(0, 2*(cfg.wing.y_attach_mm - clearance_mm));
if max_span <= 0
    error('No positive tail span fits inside y_attach=%.2f mm with clearance %.2f mm.', cfg.wing.y_attach_mm, clearance_mm);
end

base_span = base_cfg.curled_tail.span_mm;
base_arc  = base_cfg.curled_tail.arc_chord_mm;
safe_span = min(base_span, max_span);

cfg.curled_tail.enabled = true;
cfg.curled_tail.span_mm = safe_span;
cfg.curled_tail.arc_chord_mm = base_arc * area_scale;
cfg.curled_tail.curl_deg = curl_deg;
cfg.curled_tail.incidence_deg = incidence_deg;
cfg.curled_tail.scale_note = sprintf('safe span %.2f mm, arc scaled %.3gx, clearance %.1f mm', safe_span, area_scale, clearance_mm);

% Diagnostics useful for tables.
cfg.curled_tail.safe_area_scale_relative_to_safe_baseline = area_scale;
cfg.curled_tail.safe_span_clip_factor = safe_span / max(base_span,1e-12);
cfg.curled_tail.actual_area_scale_relative_to_original = (safe_span * cfg.curled_tail.arc_chord_mm) / max(base_span * base_arc, 1e-12);
end
