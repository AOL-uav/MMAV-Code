function [panels, meta] = mmav_make_opt0615_optimized_wing_panels(w)
%MMAV_MAKE_OPT0615_OPTIMIZED_WING_PANELS Build exposed wing panels from opt_0615 geometry.
%
% This replaces the simple trapezoid panel builder when
%   cfg.wing.source = 'opt0615_optimized'
%
% The opt_0615 geometry is first built in the old wing-local frame where
% +x is aft from the unshifted root LE.  Then a rigid translation places the
% root LE into the MMAV body/build frame B.
%
% Body frame B:
%   x_B aft, y_B right, z_B up
%   origin: fuselage nose centerline
%
% Required/used fields in w:
%   .origin_B_mm        : [x y z] body-frame location of unshifted root LE
%   .y_attach_mm        : exposed panel starts at |y| >= y_attach
%   .Nx, .Ny_half       : VLM mesh resolution
%   .opt0615.x_final    : optimization vector from opt_0615/cg.m
%
% Output panels use the same generic panel struct as mmav_make_wing_panels().

if ~w.enabled
    panels = struct([]);
    meta = struct();
    return;
end

if ~isfield(w,'opt0615') || ~isfield(w.opt0615,'x_final')
    error('cfg.wing.opt0615.x_final is required for source = opt0615_optimized.');
end

% Reconstruct optimized geometry parameters from opt_0615.
[p, cg_local, design, optcfg] = local_reconstruct_opt0615_p(w); %#ok<ASGLU>

halfspan_mm = 0.5 * p.b * 1000;
attach_eta = w.y_attach_mm / max(halfspan_mm, 1e-12);
attach_eta = min(max(attach_eta, 0), 0.999);

% Keep only exposed panels.  The plan_eta tables remain global eta, so only
% eta0/eta1 must be changed.
p.plan_segments(1).eta0 = attach_eta;
p.plan_segments(1).eta1 = 1.0;
p.plan_segments(1).Ny   = max(1, round(w.Ny_half));
p.Nx = max(1, round(w.Nx));
p.Ny_half = max(1, round(w.Ny_half));

wing = wing_geometry_build_flat_wing(p);

panels = repmat(base_panel_struct(), 0, 1);
idx = 0;
origin_m = reshape(w.origin_B_mm,1,3) * 1e-3;

for s = 1:numel(wing.surfaces)
    S = wing.surfaces{s};
    ps = S.panels;
    if isempty(ps)
        continue;
    end

    % The first surface from opt_0615 is right wing, the second is mirrored left.
    if s == 1
        surface_name = 'wing_R';
    elseif s == 2
        surface_name = 'wing_L';
    else
        surface_name = sprintf('wing_%d', s);
    end

    for k = 1:numel(ps)
        idx = idx + 1;
        P = ps(k);
        pnel = base_panel_struct();
        pnel.component = 'wing';
        pnel.surface = surface_name;
        pnel.i_span = P.i;
        pnel.j_chord = P.j;
        pnel.p1_B_m = P.p1 + origin_m;
        pnel.p2_B_m = P.p2 + origin_m;
        pnel.p3_B_m = P.p3 + origin_m;
        pnel.p4_B_m = P.p4 + origin_m;
        pnel = mmav_enrich_panel_geometry(pnel, [0 0 1]);
        panels(idx,1) = pnel; %#ok<AGROW>
    end
end

% Useful geometry metadata.
meta = struct();
meta.source = 'opt0615_optimized';
meta.origin_B_mm = w.origin_B_mm;
meta.attach_eta = attach_eta;
meta.local_cg_from_design_m = cg_local;
meta.design = design;
meta.optcfg = optcfg;
meta.p = p;
meta.S_total_full_m2 = design.planform.S_total;
meta.MAC_m = design.planform.MAC;
meta.xLE_MAC_local_m = design.planform.xLE_MAC;
meta.xQC_MAC_local_m = design.planform.xQC_MAC;
meta.bref_m = p.b;
meta.Sref_exposed_m2 = sum([panels.area_m2]);
meta.root_chord_m = design.planform.c_root;
meta.tip_chord_m = design.planform.c_tip;
meta.twist_root_deg = design.twist_root_deg;
meta.twist_tip_deg = design.twist_tip_deg;
meta.washout_total_deg = design.washout_total_deg;

end

% =====================================================================
function [p, cg, design, cfg] = local_reconstruct_opt0615_p(w)
% Matches opt_0615/cg.m unless explicitly overridden.

x_final = w.opt0615.x_final(:);

c0 = local_getfield(w.opt0615, 'c0_m', 0.15);
AR = local_getfield(w.opt0615, 'AR', 16/3);

p0 = struct();
p0.b = AR*c0;
p0.c_root = c0;
p0.c_tip = c0;
p0.sweepLE_deg = 0;
p0.dihedral_deg = local_getfield(w.opt0615, 'dihedral_deg', 3);
p0.Ny_half = max(1, round(w.Ny_half));
p0.Nx = max(1, round(w.Nx));
p0.winglet_height = local_getfield(w.opt0615, 'winglet_height', 0);
p0.winglet_cant_deg = local_getfield(w.opt0615, 'winglet_cant_deg', 0);
p0.winglet_sweepLE_deg = local_getfield(w.opt0615, 'winglet_sweepLE_deg', 10);
p0.winglet_chord_ratio = local_getfield(w.opt0615, 'winglet_chord_ratio', 0.70);

Nch = local_getfield(w.opt0615, 'Nch', 5);
Ntw = local_getfield(w.opt0615, 'Ntw', 4);

cfg = struct();
cfg.p0 = p0;
cfg.eta_chord = wingopt_cosspace(Nch);
cfg.eta_twist = wingopt_cosspace(Ntw);
cfg.plan_interp = local_getfield(w.opt0615, 'plan_interp', 'pchip');
cfg.twist_interp = local_getfield(w.opt0615, 'twist_interp', 'pchip');
cfg.cg_ref = local_getfield(w.opt0615, 'cg_ref', 'mac');

etaFine = linspace(0,1,2001).';
cBaseFine = p0.c_root + (p0.c_tip - p0.c_root)*etaFine;
cfg.S_target = p0.b * trapz(etaFine, cBaseFine);

i = 0;
dv = struct();
dv.i_c_ratio    = (i+1):(i+Nch);     i = i + Nch;
dv.i_tw_root    = i+1;               i = i + 1;
dv.i_tw_delta   = (i+1):(i+(Ntw-1)); i = i + (Ntw-1);
dv.i_sweepLEdeg = i+1;               i = i + 1;
dv.i_cg_x_frac  = i+1;               i = i + 1;
dv.n = i;
cfg.dv = dv;

if numel(x_final) ~= dv.n
    error('opt0615 x_final length mismatch: expected %d, got %d.', dv.n, numel(x_final));
end

[p, cg, design] = wingopt_x_to_p_cg(x_final, cfg);
end

% =====================================================================
function p = base_panel_struct()
p = struct();
p.id = NaN;
p.component = '';
p.surface = '';
p.i_span = NaN;
p.j_chord = NaN;
p.p1_B_m = [NaN NaN NaN];
p.p2_B_m = [NaN NaN NaN];
p.p3_B_m = [NaN NaN NaN];
p.p4_B_m = [NaN NaN NaN];
p.centroid_B_m = [NaN NaN NaN];
p.area_m2 = NaN;
p.normal_B = [NaN NaN NaN];
p.chord_hat_B = [NaN NaN NaN];
p.span_hat_B = [NaN NaN NaN];
p.F_B_N = [0 0 0];
p.M_ref_B_Nm = [0 0 0];
end

% =====================================================================
function v = local_getfield(s, field, default)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default;
end
end
