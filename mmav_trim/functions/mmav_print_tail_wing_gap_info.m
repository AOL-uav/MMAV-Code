function mmav_print_tail_wing_gap_info(cfg, label)
%MMAV_PRINT_TAIL_WING_GAP_INFO Print tail/wing gap diagnostics.
if nargin < 2 || isempty(label), label = 'tail geometry'; end
info = mmav_tail_wing_gap_info(cfg);
fprintf('\n--- Tail/wing gap check: %s ---\n', label);
if ~info.ok
    fprintf('  %s\n', info.message);
    return;
end
fprintf('  wing exposed starts at |y| = %.2f mm\n', info.wing_y_attach_mm);
fprintf('  tail span = %.2f mm, halfspan = %.2f mm\n', info.tail_span_mm, info.tail_halfspan_mm);
fprintf('  clearance each side = %.2f mm\n', info.clearance_each_side_mm);
fprintf('  overlap each side   = %.2f mm\n', info.overlap_each_side_mm);
fprintf('  safe span with 3 mm side clearance = %.2f mm\n', info.max_safe_span_with_3mm_clearance_mm);
fprintf('  status: %s\n', info.message);
end
