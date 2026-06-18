function [out, gamma_out] = mmav_eval_vlm_aero_state(cfg, user_cg_B_mm, state, vlm_cfg, gamma_guess)
%MMAV_EVAL_VLM_AERO_STATE Evaluate one state using opt_0615 nonlinear VLM.
%
% This is the integration layer between:
%   1) the coarse component/CG/panel geometry model, and
%   2) opt_0615's nonlinear VLM + 2D polar solver.
%
% Tail modeling is controlled by cfg.curled_tail.aero_mode:
%   'drag_only'           tail is excluded from VLM; panel drag is added separately.
%   'vlm_lifting_surface' tail is included in the coupled VLM solve.
%
% For the fuselage-attached curled-tail concept, 'drag_only' is the intended
% physical first-order model.  The VLM lifting-tail mode is kept only as a
% diagnostic/legacy option.

if nargin < 5
    gamma_guess = [];
end
if nargin < 4 || isempty(vlm_cfg)
    vlm_cfg = mmav_default_vlm_config();
end
if nargin >= 2 && ~isempty(user_cg_B_mm)
    cfg.cg.mode = 'manual';
    cfg.cg.r_cg_B_mm = reshape(user_cg_B_mm, 1, 3);
end

% Apply geometry-changing controls before the vehicle is built.
% Wing incidence is applied after panel construction; tail deformation must
% be applied before building curled-tail panels.
control = mmav_get_control_from_state(state);
cfg = mmav_apply_tail_control_to_cfg(cfg, state);

vehicle = mmav_build_coarse_vehicle(cfg);
vehicle = mmav_apply_wing_controls_to_vehicle(vehicle, cfg, state);
r_CG_B_m = vehicle.cg.r_active_B_m;

rho = getfield_default(state, 'rho_kgpm3', 1.225);
nu  = getfield_default(state, 'nu_m2ps', 1.5e-5);
V   = getfield_default(state, 'V_mps', 5.8);
alpha_deg = getfield_default(state, 'alpha_deg', 5.0);
beta_deg  = getfield_default(state, 'beta_deg', 0.0);
pitch_deg = getfield_default(state, 'pitch_deg', 0.0);
q = 0.5*rho*V^2;

% Prepare VLM solver settings.
vlm = vlm_cfg.vlm;
if isfield(vlm,'trailing_length_factor')
    factor = vlm.trailing_length_factor;
else
    factor = 50;
end
vlm.trailing_length = factor * max(vehicle.refs.bref_m, 1e-6);

% Decide how the curled tail participates in the aero model.
tail_mode = 'off';
if isfield(cfg,'curled_tail') && isfield(cfg.curled_tail,'enabled') && cfg.curled_tail.enabled
    tail_mode = lower(char(string(getfield_default(cfg.curled_tail,'aero_mode','drag_only'))));
end

switch tail_mode
    case {'drag_only','drag','body_drag'}
        vlm_components = {'wing'};
        use_tail_drag = true;
        lift_source_name = "VLM_wing_only";
    case {'vlm_lifting_surface','vlm','lifting_surface','lifting'}
        vlm_components = {'wing','curled_tail'};
        use_tail_drag = false;
        lift_source_name = "VLM_wing_plus_tail";
    case {'off','none','disabled'}
        vlm_components = {'wing'};
        use_tail_drag = false;
        lift_source_name = "VLM_wing_only";
    otherwise
        error('Unknown cfg.curled_tail.aero_mode: %s', tail_mode);
end

% Convert selected lifting panels to opt_0615 wing struct and solve VLM.
vlm_wing = mmav_panel_vehicle_to_vlm_wing(vehicle, vlm_components);
[polar, polar_info] = mmav_load_vlm_polar(vlm_cfg);
camber_param = getfield_default(vlm_cfg, 'camber_param', []);

if isempty(vlm_wing.surfaces)
    res_lift = empty_lift_result();
    gamma_out = [];
else
    [res_lift, gamma_out] = vlm_xfoil_analyze_nonlinear_polar( ...
        vlm_wing, rho, nu, V, alpha_deg, pitch_deg, r_CG_B_m, ...
        vlm, polar, gamma_guess, beta_deg, camber_param);
end

% Drag-only curled-tail panel loads.  These are zero in VLM lifting-tail mode.
tail_drag_panels = struct([]);
tail_drag_loads = empty_aggregate_loads(r_CG_B_m);
if use_tail_drag && ~isempty(vehicle.tail_panels)
    tail_drag_panels = mmav_assign_tail_drag_loads(vehicle.tail_panels, state, cfg.curled_tail);
    tail_drag_loads = mmav_aggregate_loads_from_panels(tail_drag_panels, struct([]), r_CG_B_m);
end

% Fuselage flat-plate drag body.
aero_bodies = vehicle.aero_bodies;
if ~getfield_default(vlm_cfg,'include_fuselage_drag',true)
    for k = 1:numel(aero_bodies)
        aero_bodies(k).F_B_N = [0 0 0];
        aero_bodies(k).M_ref_B_Nm = [0 0 0];
    end
else
    axesB = mmav_wind_axes_B(state);
    eD = axesB.eD_B;
    for k = 1:numel(aero_bodies)
        aero_bodies(k).F_B_N = [0 0 0];
        aero_bodies(k).M_ref_B_Nm = [0 0 0];
        if isfield(aero_bodies(k),'enabled') && ~aero_bodies(k).enabled
            continue;
        end
        D = q * aero_bodies(k).S_m2 * aero_bodies(k).CD;
        aero_bodies(k).F_B_N = D * eD;
    end
end
fuse_loads = mmav_aggregate_loads_from_panels(struct([]), aero_bodies, r_CG_B_m);

F_lift = res_lift.F;
M_lift = res_lift.M;
F_total = F_lift + tail_drag_loads.F_B_N + fuse_loads.F_B_N;
M_total = M_lift + tail_drag_loads.M_CG_B_Nm + fuse_loads.M_CG_B_Nm;

% Coefficients using the design reference wing area/chord.
Sref = vehicle.refs.Sref_m2;
cref = vehicle.refs.cref_m;
qS = max(q*Sref, 1e-12);
qSc = max(q*Sref*cref, 1e-12);
axesB = mmav_wind_axes_B(state);

loads = struct();
loads.F_B_N = F_total;
loads.M_CG_B_Nm = M_total;
loads.r_CG_B_m = r_CG_B_m;
loads.tables.by_source = make_source_table(F_lift, M_lift, tail_drag_loads.tables.by_source, fuse_loads.tables.by_source, lift_source_name);

out = struct();
out.model = 'coarse_component + opt_0615 nonlinear VLM + configurable tail model';
out.tail_aero_mode = tail_mode;
out.vehicle = vehicle;
out.vlm_wing = vlm_wing;
out.vlm_result_lifting = res_lift;
out.polar_info = polar_info;
out.tail_drag_panels = tail_drag_panels;
out.tail_drag_loads = tail_drag_loads;
out.aero_bodies = aero_bodies;
out.fuselage_loads = fuse_loads;
out.loads = loads;
out.q_Pa = q;
out.Sref_m2 = Sref;
out.cref_m = cref;
out.CF_B = F_total / qS;
out.CM_CG_B = M_total / qSc;
out.CL_wind = dot(F_total, axesB.eL_B) / qS;
out.CD_wind = dot(F_total, axesB.eD_B) / qS;
out.Cm_y = out.CM_CG_B(2);
out.CLz = out.CF_B(3);
out.CD_x = out.CF_B(1);
out.state = state;
out.control = control;
out.vlm_cfg = vlm_cfg;

if getfield_default(vlm_cfg,'compute_separate_component_diagnostics',false)
    out.component_diagnostics = run_separate_component_diagnostics(vehicle, state, vlm, polar, vlm_cfg, r_CG_B_m, tail_mode, tail_drag_loads);
end

% Simple recognizable aero-state diagnostic line.  Enable with either:
%   vlm_cfg.diagnostics = true;
%   vlm_cfg.diagnostics_aero_state = true;
%   state.diagnostics = true;
%   cfg.diagnostics = true;
if mmav_diag_enabled(cfg, vlm_cfg, state, 'diagnostics_aero_state')
    lift_ok = getfield_default(res_lift, 'converged', false);
    lift_err = getfield_default(res_lift, 'err', NaN);
    lift_iter = getfield_default(res_lift, 'iter', NaN);
    fprintf(['[VLM-AERO] ', ...
        'mode=%s V=%.3f alpha=%.3f beta=%.3f CG=[%.1f %.1f %.1f]mm | ', ...
        'ctrl wingC=%+.2f wingD=%+.2f R=%+.2f L=%+.2f tailCurl=%+.2f | ', ...
        'F=[%+.3f %+.3f %+.3f]N M=[%+.4f %+.4f %+.4f]Nm | ', ...
        'CL=%.4f CD=%.4f Cm=%+.4f | ', ...
        'liftOK=%d err=%.2e iter=%g | ', ...
        'tailF=[%+.3f %+.3f %+.3f]N fuseF=[%+.3f %+.3f %+.3f]N\n'], ...
        tail_mode, V, alpha_deg, beta_deg, 1000*r_CG_B_m(1), 1000*r_CG_B_m(2), 1000*r_CG_B_m(3), ...
        control.wing_collective_deg, control.wing_differential_deg, control.wing_R_incidence_deg, control.wing_L_incidence_deg, control.tail_curl_delta_deg, ...
        F_total(1), F_total(2), F_total(3), M_total(1), M_total(2), M_total(3), ...
        out.CL_wind, out.CD_wind, out.Cm_y, ...
        lift_ok, lift_err, lift_iter, ...
        tail_drag_loads.F_B_N(1), tail_drag_loads.F_B_N(2), tail_drag_loads.F_B_N(3), ...
        fuse_loads.F_B_N(1), fuse_loads.F_B_N(2), fuse_loads.F_B_N(3));
end

end

% =====================================================================
function loads = empty_aggregate_loads(r_CG_B_m)
loads = struct();
loads.F_B_N = [0 0 0];
loads.M_CG_B_Nm = [0 0 0];
loads.r_CG_B_m = r_CG_B_m;
loads.tables.by_source = table(strings(0,1), strings(0,1), zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), ...
    'VariableNames', {'source_name','source_type','Fx','Fy','Fz','Mx','My','Mz'});
end

% =====================================================================
function res = empty_lift_result()
res = struct();
res.F = [0 0 0];
res.M = [0 0 0];
res.CL = NaN;
res.CD = NaN;
res.Cm_y = NaN;
res.converged = true;
res.err = 0;
res.iter = 0;
end

% =====================================================================
function T = make_source_table(F_lift, M_lift, tail_T, fuse_T, lift_source_name)
if nargin < 5 || isempty(lift_source_name)
    lift_source_name = "VLM_lifting_surfaces";
end
source_name = lift_source_name;
source_type = "nonlinear_vlm";
Fx = F_lift(1); Fy = F_lift(2); Fz = F_lift(3);
Mx = M_lift(1); My = M_lift(2); Mz = M_lift(3);
T = table(source_name, source_type, Fx, Fy, Fz, Mx, My, Mz);
if nargin >= 3 && ~isempty(tail_T)
    T = [T; tail_T];
end
if nargin >= 4 && ~isempty(fuse_T)
    T = [T; fuse_T];
end
end

% =====================================================================
function diagT = run_separate_component_diagnostics(vehicle, state, vlm, polar, vlm_cfg, r_CG_B_m, tail_mode, tail_drag_loads)
% Approximate component diagnostics from separate solves/loads.
rho = getfield_default(state, 'rho_kgpm3', 1.225);
nu  = getfield_default(state, 'nu_m2ps', 1.5e-5);
V   = getfield_default(state, 'V_mps', 5.8);
alpha_deg = getfield_default(state, 'alpha_deg', 5.0);
beta_deg  = getfield_default(state, 'beta_deg', 0.0);
pitch_deg = getfield_default(state, 'pitch_deg', 0.0);
camber_param = getfield_default(vlm_cfg, 'camber_param', []);

names = ["wing"; "curled_tail"];
Fx = NaN(2,1); Fy = NaN(2,1); Fz = NaN(2,1);
Mx = NaN(2,1); My = NaN(2,1); Mz = NaN(2,1);
converged = false(2,1);
model = strings(2,1);

% Wing-only VLM.
try
    w = mmav_panel_vehicle_to_vlm_wing(vehicle, 'wing');
    if ~isempty(w.surfaces)
        [r,~] = vlm_xfoil_analyze_nonlinear_polar(w, rho, nu, V, alpha_deg, pitch_deg, r_CG_B_m, ...
            vlm, polar, [], beta_deg, camber_param);
        Fx(1) = r.F(1); Fy(1) = r.F(2); Fz(1) = r.F(3);
        Mx(1) = r.M(1); My(1) = r.M(2); Mz(1) = r.M(3);
        converged(1) = r.converged;
        model(1) = "separate_vlm";
    end
catch ME
    warning('Separate wing VLM diagnostic failed: %s', ME.message);
end

% Tail diagnostic follows the selected tail mode.
if any(strcmpi(tail_mode, {'drag_only','drag','body_drag'}))
    if ~isempty(tail_drag_loads) && isfield(tail_drag_loads,'F_B_N')
        Fx(2) = tail_drag_loads.F_B_N(1); Fy(2) = tail_drag_loads.F_B_N(2); Fz(2) = tail_drag_loads.F_B_N(3);
        Mx(2) = tail_drag_loads.M_CG_B_Nm(1); My(2) = tail_drag_loads.M_CG_B_Nm(2); Mz(2) = tail_drag_loads.M_CG_B_Nm(3);
        converged(2) = true;
        model(2) = "panel_drag_only";
    end
else
    try
        w = mmav_panel_vehicle_to_vlm_wing(vehicle, 'curled_tail');
        if ~isempty(w.surfaces)
            [r,~] = vlm_xfoil_analyze_nonlinear_polar(w, rho, nu, V, alpha_deg, pitch_deg, r_CG_B_m, ...
                vlm, polar, [], beta_deg, camber_param);
            Fx(2) = r.F(1); Fy(2) = r.F(2); Fz(2) = r.F(3);
            Mx(2) = r.M(1); My(2) = r.M(2); Mz(2) = r.M(3);
            converged(2) = r.converged;
            model(2) = "separate_vlm";
        end
    catch ME
        warning('Separate tail VLM diagnostic failed: %s', ME.message);
    end
end

component = names;
diagT = table(component, model, Fx, Fy, Fz, Mx, My, Mz, converged);
end


% =====================================================================
function tf = mmav_diag_enabled(cfg, vlm_cfg, state, specific_field)
%MMAV_DIAG_ENABLED Small local flag reader for optional console diagnostics.
% Checks both generic and function-specific flags, without requiring any
% particular config schema.
tf = false;
tf = tf || mmav_local_flag(vlm_cfg, 'diagnostics');
tf = tf || mmav_local_flag(vlm_cfg, specific_field);
tf = tf || mmav_local_flag(state, 'diagnostics');
tf = tf || mmav_local_flag(state, specific_field);
tf = tf || mmav_local_flag(cfg, 'diagnostics');
tf = tf || mmav_local_flag(cfg, specific_field);
if isstruct(vlm_cfg) && isfield(vlm_cfg,'diag')
    tf = tf || mmav_local_flag(vlm_cfg.diag, 'enabled');
    tf = tf || mmav_local_flag(vlm_cfg.diag, specific_field);
end
if isstruct(cfg) && isfield(cfg,'diag')
    tf = tf || mmav_local_flag(cfg.diag, 'enabled');
    tf = tf || mmav_local_flag(cfg.diag, specific_field);
end
end

% =====================================================================
function tf = mmav_local_flag(s, field)
tf = false;
if ~isstruct(s) || ~isfield(s, field) || isempty(s.(field))
    return;
end
v = s.(field);
if islogical(v) || isnumeric(v)
    tf = any(v(:) ~= 0);
elseif ischar(v) || isstring(v)
    try
        txt = lower(strtrim(char(string(v))));
        tf = any(strcmp(txt, {'true','on','yes','1','enable','enabled'}));
    catch
        tf = false;
    end
elseif isstruct(v)
    tf = mmav_local_flag(v, 'enabled');
end
end

% =====================================================================
function v = getfield_default(s, field, default)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end
