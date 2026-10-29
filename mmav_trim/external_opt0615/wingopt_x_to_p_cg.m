function [p, cg, design] = wingopt_x_to_p_cg(x, cfg)
%WINGOPT_X_TO_P_CG  Map design vector -> geometry parameters p + cg.
%
% Design vector definition:
%   x = [ c_ratio(1:Nch),
%         twist_root,
%         twist_delta(1:Ntw-1),
%         sweepLE_deg,
%         cg_x_frac ]
%
% CG reference can be configured with cfg.cg_ref:
%   'root_chord' (legacy default) : cg_x = cg_x_frac * c_root
%   'mac'                        : cg_x = xLE_MAC + cg_x_frac * MAC

p0 = cfg.p0;
dv = cfg.dv;

% --- unpack ---
c_ratio    = x(dv.i_c_ratio);
tw_root    = x(dv.i_tw_root);
tw_delta   = x(dv.i_tw_delta);
sweepLEdeg = x(dv.i_sweepLEdeg);
cg_x_frac  = x(dv.i_cg_x_frac);

% --- chord distribution (control points) ---
eta_cp = cfg.eta_chord(:);

% baseline chord distribution for scaling (linear-taper baseline)
c_base = p0.c_root + (p0.c_tip - p0.c_root) * eta_cp;

% raw chord at control points
c_raw = max(c_base(:) .* c_ratio(:), 1e-4);

% fixed-area scaling
etaFine_area = linspace(0, 1, 1201).';
cFine_area = interp1(eta_cp, c_raw, etaFine_area, cfg.plan_interp, 'extrap');
cFine_area = max(cFine_area, 1e-4);
I = trapz(etaFine_area, cFine_area);      % integral_0^1 c(eta) d eta
S_raw = p0.b * I;                         % total area for symmetric wing
scale = cfg.S_target / max(S_raw, 1e-12);

c_cp = c_raw * scale;

% --- leading edge x (constant sweep for now) ---
halfspan = p0.b / 2;
y_cp = eta_cp * halfspan;
xLE_cp = tan(deg2rad(sweepLEdeg)) * y_cp;
xTE_cp = xLE_cp + c_cp;

% --- twist distribution ---
eta_tw = cfg.eta_twist(:);
Ntw = numel(eta_tw);
if numel(tw_delta) ~= (Ntw - 1)
    error('twist_delta length mismatch: expected %d, got %d.', Ntw-1, numel(tw_delta));
end

tw_cp = tw_root + [0; cumsum(tw_delta(:))];

% --- planform metrics on a uniform spanwise grid ---
planform = wingopt_planform_metrics(p0.b, eta_cp, xLE_cp, xTE_cp, cfg.plan_interp);

% --- assemble p struct (single segment) ---
p = p0;

p.plan_segments = struct();
p.plan_segments(1).eta0        = 0.0;
p.plan_segments(1).eta1        = 1.0;
p.plan_segments(1).plan_eta    = eta_cp;
p.plan_segments(1).plan_xLE    = xLE_cp;
p.plan_segments(1).plan_xTE    = xTE_cp;
p.plan_segments(1).plan_interp = cfg.plan_interp;

p.c_root = c_cp(1);
p.c_tip  = c_cp(end);
p.sweepLE_deg = sweepLEdeg;

p.twist_eta    = eta_tw;
p.twist_deg    = tw_cp;
p.twist_interp = cfg.twist_interp;

% --- cg reference ---
if isfield(cfg, 'cg_ref') && ~isempty(cfg.cg_ref)
    cg_ref = char(string(cfg.cg_ref));
else
    cg_ref = 'root_chord';
end

switch lower(strtrim(cg_ref))
    case {'root', 'root_chord', 'croot'}
        cg_x = cg_x_frac * p.c_root;
    case {'mac', 'lemac'}
        cg_x = planform.xLE_MAC + cg_x_frac * planform.MAC;
    otherwise
        error('Unsupported cfg.cg_ref ''%s''. Use ''root_chord'' or ''mac''.', cg_ref);
end

cg = [cg_x, 0.0, 0.0];

% --- smoothness metrics on a uniform eta grid (removes cosine-grid bias) ---
etaFine_reg = linspace(0, 1, 401).';
h = etaFine_reg(2) - etaFine_reg(1);

cFine_reg = interp1(eta_cp, c_cp, etaFine_reg, cfg.plan_interp, 'extrap');
cFine_reg = max(cFine_reg, 1e-4);
d2c = diff(cFine_reg, 2) / (h^2);
chord_smooth = trapz(etaFine_reg(2:end-1), d2c.^2) / (mean(cFine_reg)^2 + 1e-12);

twFine_reg = interp1(eta_tw, tw_cp, etaFine_reg, cfg.twist_interp, 'extrap');
d2t = diff(twFine_reg, 2) / (h^2);
twist_scale = max(max(abs(twFine_reg)), 1.0);
twist_smooth = trapz(etaFine_reg(2:end-1), d2t.^2) / (twist_scale^2 + 1e-12);

design = struct();
design.eta_cp = eta_cp;
design.c_cp = c_cp;
design.xLE_cp = xLE_cp;
design.xTE_cp = xTE_cp;
design.tw_cp = tw_cp;
design.eta_tw = eta_tw;
design.scale_area = scale;
design.cg_x_frac = cg_x_frac;
design.cg_ref = cg_ref;
design.cg_x = cg_x;
design.chord_smooth = chord_smooth;
design.twist_smooth = twist_smooth;
design.twist_root_deg = tw_cp(1);
design.twist_tip_deg = tw_cp(end);
design.washout_total_deg = tw_cp(end) - tw_cp(1);
design.planform = planform;

end
