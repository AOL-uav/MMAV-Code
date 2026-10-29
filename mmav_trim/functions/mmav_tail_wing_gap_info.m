function info = mmav_tail_wing_gap_info(cfg)
%MMAV_TAIL_WING_GAP_INFO Diagnose lateral overlap between curled tail and wing.
%
% The opt_0615 wing panel model removes the inboard region |y| < y_attach.
% A centerline curled tail should usually stay inside that gap unless the CAD
% intentionally allows overlap/interaction.  This helper reports the available
% half-gap and whether the current tail span intrudes into exposed wing panels.

info = struct();

if ~isfield(cfg,'wing') || ~isfield(cfg.wing,'y_attach_mm')
    info.ok = false;
    info.message = 'cfg.wing.y_attach_mm is missing.';
    return;
end
if ~isfield(cfg,'curled_tail') || ~isfield(cfg.curled_tail,'span_mm')
    info.ok = false;
    info.message = 'cfg.curled_tail.span_mm is missing.';
    return;
end

y_attach = cfg.wing.y_attach_mm;
tail_halfspan = 0.5 * cfg.curled_tail.span_mm;

info.wing_y_attach_mm = y_attach;
info.tail_span_mm = cfg.curled_tail.span_mm;
info.tail_halfspan_mm = tail_halfspan;
info.clearance_each_side_mm = y_attach - tail_halfspan;
info.overlap_each_side_mm = max(0, tail_halfspan - y_attach);
info.has_overlap = info.overlap_each_side_mm > 1e-9;
info.max_safe_span_no_clearance_mm = 2*y_attach;
info.max_safe_span_with_3mm_clearance_mm = max(0, 2*(y_attach - 3));
info.max_safe_span_with_5mm_clearance_mm = max(0, 2*(y_attach - 5));
info.ok = true;
if info.has_overlap
    info.message = sprintf('Tail halfspan exceeds wing exposed-panel gap by %.2f mm per side.', info.overlap_each_side_mm);
else
    info.message = sprintf('Tail stays inside wing gap with %.2f mm clearance per side.', info.clearance_each_side_mm);
end
end
