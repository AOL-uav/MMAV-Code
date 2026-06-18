function cfg = mmav_apply_tail_control_to_cfg(cfg, state)
%MMAV_APPLY_TAIL_CONTROL_TO_CFG Apply commanded tail deformation to cfg.
%
% The fuselage-attached tail deformation is modeled as a change in curl angle
% and, optionally, incidence angle.  This affects the generated tail panel
% geometry before drag-only panel loads are computed.

ctrl = mmav_get_control_from_state(state);
if ~isfield(cfg,'curled_tail') || ~isfield(cfg.curled_tail,'enabled') || ~cfg.curled_tail.enabled
    return;
end

cfg.curled_tail.base_curl_deg = cfg.curled_tail.curl_deg;
cfg.curled_tail.base_incidence_deg = cfg.curled_tail.incidence_deg;

cfg.curled_tail.curl_deg = cfg.curled_tail.curl_deg + ctrl.tail_curl_delta_deg;
cfg.curled_tail.incidence_deg = cfg.curled_tail.incidence_deg + ctrl.tail_incidence_delta_deg;

cfg.curled_tail.control_tail_curl_delta_deg = ctrl.tail_curl_delta_deg;
cfg.curled_tail.control_tail_incidence_delta_deg = ctrl.tail_incidence_delta_deg;
cfg.curled_tail.controlled_curl_deg = cfg.curled_tail.curl_deg;
cfg.curled_tail.controlled_incidence_deg = cfg.curled_tail.incidence_deg;

if isfield(cfg.curled_tail,'design_note')
    cfg.curled_tail.design_note = sprintf('%s, ctrl dCurl %.2f deg, ctrl dInc %.2f deg', ...
        cfg.curled_tail.design_note, ctrl.tail_curl_delta_deg, ctrl.tail_incidence_delta_deg);
end
end
