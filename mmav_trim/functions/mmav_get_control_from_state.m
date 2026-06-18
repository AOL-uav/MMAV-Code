function ctrl = mmav_get_control_from_state(state)
%MMAV_GET_CONTROL_FROM_STATE Normalize trim/control fields from state.
%
% Supported fields:
%   state.control.wing_collective_deg
%   state.control.wing_differential_deg
%   state.control.wing_R_incidence_deg
%   state.control.wing_L_incidence_deg
%   state.control.tail_curl_delta_deg
%   state.control.tail_incidence_delta_deg
%
% Direct R/L wing incidence overrides collective/differential if both are
% provided.  Otherwise:
%   delta_R = collective + differential
%   delta_L = collective - differential
%
% For symmetric longitudinal trim, use differential = 0.

ctrl = struct();
ctrl.enabled = false;
ctrl.wing_collective_deg = 0.0;
ctrl.wing_differential_deg = 0.0;
ctrl.wing_R_incidence_deg = NaN;
ctrl.wing_L_incidence_deg = NaN;
ctrl.tail_curl_delta_deg = 0.0;
ctrl.tail_incidence_delta_deg = 0.0;
ctrl.note = 'positive wing incidence uses +Ry rotation about spanwise hinge; positive tail delta increases curl_deg';

if nargin < 1 || isempty(state) || ~isstruct(state)
    ctrl.wing_R_incidence_deg = 0.0;
    ctrl.wing_L_incidence_deg = 0.0;
    return;
end

s = struct();
if isfield(state,'control') && isstruct(state.control)
    s = state.control;
elseif isfield(state,'controls') && isstruct(state.controls)
    s = state.controls;
end

% Also allow top-level shorthand fields for quick tests.
fields = {'wing_collective_deg','wing_differential_deg', ...
          'wing_R_incidence_deg','wing_L_incidence_deg', ...
          'tail_curl_delta_deg','tail_incidence_delta_deg'};
for k = 1:numel(fields)
    f = fields{k};
    if isfield(s,f) && ~isempty(s.(f))
        ctrl.(f) = s.(f);
    elseif isfield(state,f) && ~isempty(state.(f))
        ctrl.(f) = state.(f);
    end
end

if ~isfinite(ctrl.wing_R_incidence_deg) || ~isfinite(ctrl.wing_L_incidence_deg)
    ctrl.wing_R_incidence_deg = ctrl.wing_collective_deg + ctrl.wing_differential_deg;
    ctrl.wing_L_incidence_deg = ctrl.wing_collective_deg - ctrl.wing_differential_deg;
else
    ctrl.wing_collective_deg = 0.5*(ctrl.wing_R_incidence_deg + ctrl.wing_L_incidence_deg);
    ctrl.wing_differential_deg = 0.5*(ctrl.wing_R_incidence_deg - ctrl.wing_L_incidence_deg);
end

vals = [ctrl.wing_collective_deg, ctrl.wing_differential_deg, ...
        ctrl.wing_R_incidence_deg, ctrl.wing_L_incidence_deg, ...
        ctrl.tail_curl_delta_deg, ctrl.tail_incidence_delta_deg];
ctrl.enabled = any(isfinite(vals) & abs(vals) > 1e-12);
end
