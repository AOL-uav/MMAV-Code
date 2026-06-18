function [res, gamma_out] = vlm_xfoil_analyze_nonlinear_polar( ...
    wing, rho, nu, V, alpha_deg, pitch_deg, cg, vlm, polar, gamma_guess, varargin)
%VLM_XFOIL_ANALYZE_NONLINEAR_POLAR
%
% Quasi-nonlinear VLM + 2D XFOIL polars:
%   - VLM provides 3D induced velocities / induced drag
%   - XFOIL polar lookup (alpha, Re) provides sectional Cl/Cd/Cm
%   - Fixed-point iteration couples them via:
%         Gamma_section = 0.5 * |V_perp| * c * Cl_2D(alpha_eff, Re)
%
% This function is extracted (with minimal changes) from:
%   demo_flat_wing_vlm_xfoil_coupled_v4_Nxrobust.m
%
% Major difference vs the original monolithic demo:
%   - 2D polar interpolation is delegated to xfoil_polar_lookup().
%
% Inputs
%   wing        : wing geometry struct
%   rho         : density [kg/m^3]
%   nu          : kinematic viscosity [m^2/s]
%   V           : freestream speed [m/s]
%   alpha_deg   : angle of attack [deg]
%   pitch_deg   : pitch rotation about CG [deg] (geometry rotation)
%   cg          : reference CG [x y z]
%   vlm         : settings struct (see demo)
%   polar       : struct from xfoil_polar_load
%   gamma_guess : optional warm-start circulation vector
%
% Outputs
%   res       : result struct with coefficients, diagnostics
%   gamma_out : converged circulation (for warm-start)


% ------------------ Optional inputs ------------------
% Additional optional arguments (via varargin):
%   beta_deg      : sideslip angle [deg] (default 0)
%   camber_param  : scalar camber/flap parameter for polar family (default [])
%
% Calling patterns supported:
%   [res,gam] = vlm_xfoil_analyze_nonlinear_polar(..., gamma_guess)
%   [res,gam] = vlm_xfoil_analyze_nonlinear_polar(..., gamma_guess, beta_deg)
%   [res,gam] = vlm_xfoil_analyze_nonlinear_polar(..., gamma_guess, beta_deg, camber_param)

beta_deg = 0;
camber_param = [];
if ~isempty(varargin)
    beta_deg = varargin{1};
    if numel(varargin) >= 2
        camber_param = varargin{2};
    end
end

% ------------------ 1) assemble panels with surface id ------------------
panels = [];
for s = 1:numel(wing.surfaces)
    ps = wing.surfaces{s}.panels;
    for k = 1:numel(ps)
        ps(k).surf = s;
    end
    panels = [panels; ps];
end
N = numel(panels);

% ------------------ 2) apply pitch rotation about CG ------------------
R = rot_y(deg2rad(pitch_deg));
for k = 1:N
    panels(k) = rotate_panel_about_point(panels(k), R, cg);
end

% ------------------ 3) freestream ------------------
alpha = deg2rad(alpha_deg);
beta  = deg2rad(beta_deg);

% Freestream direction (body axes: x forward, y right, z up)
Vinf  = V * [cos(alpha)*cos(beta), cos(alpha)*sin(beta), sin(alpha)];
Vdir  = Vinf / (norm(Vinf) + 1e-12);

% ------------------ 4) build connected rings ------------------
[cp, nrm, rings, idxSurf] = build_connected_rings(panels, Vdir, vlm.trailing_length);

% ------------------ 5) linear VLM solution for initial shape ------------------
A   = zeros(N,N);
rhs = zeros(N,1);
for i = 1:N
    rhs(i) = -dot(nrm(i,:), Vinf);
    for j = 1:N
        vij = induced_velocity_ring(cp(i,:), rings(j));
        A(i,j) = dot(nrm(i,:), vij);
    end
end

gamma_lin = A \ rhs;

% Initial guess
if nargin >= 10 && ~isempty(gamma_guess) && numel(gamma_guess) == N
    gamma = gamma_guess;
else
    gamma = gamma_lin;
end

% Build strip definitions + chordwise shape vectors (per strip)
[strips, strip_shape] = build_strips_and_shape(panels, rings, idxSurf, gamma);

% ------------------ 6) fixed-point iteration ------------------
max_iter = vlm.nl.max_iter;
relax    = vlm.nl.relax;
tol      = vlm.nl.tol;
verbose  = isfield(vlm.nl,'verbose') && vlm.nl.verbose;

% Convergence scaling (robust near zero-lift)
c_list = zeros(numel(strips),1);
for sidx = 1:numel(strips)
    le = strips(sidx).le_idx;
    te = strips(sidx).te_idx;
    Ple = panels(le);
    Pte = panels(te);
    LE = 0.5*(Ple.p1 + Ple.p2);
    TE = 0.5*(Pte.p3 + Pte.p4);
    c_list(sidx) = norm(TE - LE);
end
c_ref = mean(c_list(c_list > 1e-9));
if ~isfinite(c_ref) || c_ref < 1e-9
    c_ref = 1.0;
end
gamma_scale = 0.5 * V * c_ref;

% Adaptive relaxation safeguards
relax_cur = relax;
relax_min = 0.005;
relax_max = max(0.25, relax);
err_prev  = inf;

% Stagnation detection
stagnated   = false;
err_best    = inf;
stall_count = 0;

% Diagnostics arrays (per strip)
alpha_eff_deg   = NaN(numel(strips),1);
alpha_polar_deg = NaN(numel(strips),1);
Re_loc          = NaN(numel(strips),1);
Cl2d            = NaN(numel(strips),1);
Cd2d            = NaN(numel(strips),1);
Cm2d            = NaN(numel(strips),1);
Vsec_mag        = NaN(numel(strips),1);
sec_axes        = repmat(struct('c_hat',[1 0 0],'n_hat',[0 0 1],'s_hat',[0 1 0],'Vsec_dir',[1 0 0]), numel(strips), 1);

err = inf;
for it = 1:max_iter
    gamma_prev = gamma;

    % update gamma from polars strip-by-strip
    gamma_target = gamma;
    for sidx = 1:numel(strips)
        le = strips(sidx).le_idx;
        te = strips(sidx).te_idx;

        Ple = panels(le);
        Pte = panels(te);

        % full-strip chord vector (LE mid -> TE mid), robust for Nx>1
        LE = 0.5*(Ple.p1 + Ple.p2);
        TE = 0.5*(Pte.p3 + Pte.p4);
        chord_vec = TE - LE;
        c = norm(chord_vec);
        if c < 1e-9
            continue;
        end

        % --- Local section axes (consistent across mirrored halves) ---
        qc_in  = Ple.p1 + 0.25*(Pte.p4 - Ple.p1);
        qc_out = Ple.p2 + 0.25*(Pte.p3 - Ple.p2);

        bvec_raw = qc_out - qc_in;
        bmag = norm(bvec_raw);
        if bmag < 1e-9
            continue;
        end
        b_hat_raw = bvec_raw / bmag;

        % force span axis for *polar evaluation* toward +Y
        b_hat = b_hat_raw;
        if dot(b_hat, [0 1 0]) < 0
            b_hat = -b_hat;
        end

        % sign mapping between polar-consistent span axis and actual ring orientation
        sgn_gamma = sign(dot(b_hat_raw, b_hat));
        if sgn_gamma == 0
            sgn_gamma = 1;
        end

        % chord axis in the section plane
        c_hat = chord_vec / c;
        c_sec = c_hat - dot(c_hat, b_hat) * b_hat;
        c_sec = c_sec / (norm(c_sec) + 1e-12);

        % section normal: choose sign so +alpha_eff -> +Cl
        n_sec = cross(b_hat, c_sec);
        n_sec = n_sec / (norm(n_sec) + 1e-12);
        if dot(n_sec, [0 0 1]) < 0
            n_sec = -n_sec;
        end

        % Cm axis about spanwise direction
        s_sec = b_hat;
        s_sec = s_sec / (norm(s_sec) + 1e-12);

        % freestream components in section plane
        Vc_inf = dot(Vinf, c_sec);
        Vn_inf = dot(Vinf, n_sec);

        % --- induced velocity at a physical strip evaluation point ---
        ev_in  = Ple.p1 + 0.75*(Pte.p4 - Ple.p1);
        ev_out = Ple.p2 + 0.75*(Pte.p3 - Ple.p2);
        eval_pt = 0.5*(ev_in + ev_out);

        v_ind = [0 0 0];
        for j = 1:N
            v_ind = v_ind + gamma(j) * induced_velocity_ring(eval_pt, rings(j));
        end

        Vn_ind = dot(v_ind, n_sec);
        k_downwash = 0.25;            % tune ~0.8..1.2
        Vn_ind = k_downwash * Vn_ind;
        % AoA used for polar lookup (ignore induced chordwise component)
        a_polar = atan2(Vn_inf + Vn_ind, Vc_inf);

        a_polar_deg = rad2deg(a_polar);

        % local flow at strip evaluation point
        Vloc = Vinf + v_ind;

        % project into section plane
        Vsec = Vloc - dot(Vloc, b_hat) * b_hat;
        Vsm  = norm(Vsec);
        if Vsm < 1e-9
            continue;
        end

        % Local effective AoA in section plane (for reporting)
        a_eff = atan2(dot(Vsec, n_sec), dot(Vsec, c_sec));
        a_eff_deg = rad2deg(a_eff);

        Re = (Vsm * c) / max(nu, 1e-12);

        % --- Polar lookup (table-lookup module) ---
        [cl, cd, cm] = xfoil_polar_lookup(polar, a_polar_deg, Re, camber_param);

        % circulation target for this strip
        Gamma_target = sgn_gamma * (0.5 * Vsm * c * cl);

        % chordwise distribution shape (TE gamma equals 1)
        sh = strip_shape{sidx};
        gamma_target(strips(sidx).idx) = Gamma_target * sh;

        % save for output
        alpha_eff_deg(sidx)   = a_eff_deg;
        alpha_polar_deg(sidx) = a_polar_deg;
        Re_loc(sidx)          = Re;
        Cl2d(sidx)            = cl;
        Cd2d(sidx)            = cd;
        Cm2d(sidx)            = cm;
        Vsec_mag(sidx)        = Vsm;
        sec_axes(sidx).c_hat    = c_sec;
        sec_axes(sidx).n_hat    = n_sec;
        sec_axes(sidx).s_hat    = s_sec;
        sec_axes(sidx).Vsec_dir = Vsec / Vsm;
        sec_axes(sidx).c_len    = c;
        sec_axes(sidx).dy       = bmag;
        sec_axes(sidx).qc_mid   = 0.5*(qc_in + qc_out);
    end

    % fixed-point residual
    err = norm(gamma_target - gamma_prev, inf) / max(gamma_scale, norm(gamma_prev, inf));

    % oscillation detection (sign flips)
    flip_frac = mean((gamma_target .* gamma_prev) < 0);
    if flip_frac > 0.30
        relax_cur = max(relax_cur * 0.5, relax_min);
    end

    % track best residual and stagnation
    if err < err_best
        err_best = err;
        stall_count = 0;
    else
        stall_count = stall_count + 1;
    end
    if stall_count > 200
        stagnated = true;
        break;
    end

    % adaptive relaxation
    if it > 1
        if err > 1.05*err_prev
            relax_cur = max(relax_cur * 0.5, relax_min);
        elseif err < 0.70*err_prev
            relax_cur = min(relax_cur * 1.05, relax_max);
        end
    end

    gamma = gamma_prev + relax_cur * (gamma_target - gamma_prev);

    if verbose
        fprintf('  it=%d err=%.3e  relax=%.3f\n', it, err, relax_cur);
    end
    if err < tol
        break;
    end
    err_prev = err;
end

iter_used = it;
converged = isfinite(err) && (err < tol);

% ------------------ 7) forces: induced (KJ) + profile drag + Cm ------------------

% induced velocity at bound midpoints
Vind_b = zeros(N,3);
for i = 1:N
    v = [0 0 0];
    for j = 1:N
        v = v + gamma(j) * induced_velocity_ring(rings(i).bound_mid, rings(j));
    end
    Vind_b(i,:) = v;
end

F_ind = [0 0 0];
M_ind = [0 0 0];

for i = 1:N
    % robust upstream index in same surface/span strip
    s  = panels(i).surf;
    ii = panels(i).i;
    jj = panels(i).j;
    map = idxSurf{s};

    if jj == 1
        G_net = gamma(i);
    else
        i_up = map(ii, jj-1);
        G_net = gamma(i) - gamma(i_up);
    end

    Vlocal = Vinf + Vind_b(i,:);
    Fi = rho * G_net * cross(Vlocal, rings(i).bound_vec);
    ri = rings(i).bound_mid - cg;
    Mi = cross(ri, Fi);

    F_ind = F_ind + Fi;
    M_ind = M_ind + Mi;
end

% profile drag + pitching moment (integrated per strip)
F_prof = [0 0 0];
M_prof = [0 0 0];

for sidx = 1:numel(strips)
    ax = sec_axes(sidx);
    if ~isfield(ax,'c_len') || ax.c_len < 1e-9
        continue;
    end
    c  = ax.c_len;
    dy = ax.dy;
    Vsm = Vsec_mag(sidx);
    if Vsm < 1e-9
        continue;
    end

    qsec = 0.5 * rho * Vsm^2;
    area_strip = c * dy;

    Dp = qsec * area_strip * Cd2d(sidx);
    Fp = Dp * ax.Vsec_dir;

    Mp_mag = qsec * area_strip * c * Cm2d(sidx);
    Mp = Mp_mag * ax.s_hat;

    r = ax.qc_mid - cg;
    M_from_force = cross(r, Fp);

    F_prof = F_prof + Fp;
    M_prof = M_prof + (M_from_force + Mp);
end

Ftot = F_ind + F_prof;
Mtot = M_ind + M_prof;

% ------------------ 8) coefficients ------------------
if isfield(vlm,'ref_area') && strcmpi(vlm.ref_area,'projected_xy')
    Sref = estimate_projected_area_xy_from_panels(panels);
else
    Sref = estimate_total_area_from_panels(panels);
end
cref = estimate_mac_main_wing(wing);
bref = estimate_span_from_panels(panels);
qinf = 0.5 * rho * V^2;

% wind axes (supports sideslip)
eV = Vinf / (norm(Vinf) + 1e-12);
eD = eV;

% Define a wind Y axis using the global/body up direction to keep it well-defined.
z_b = [0 0 1];
eY = cross(z_b, eD);
if norm(eY) < 1e-12
    eY = [0 1 0];
else
    eY = eY / (norm(eY) + 1e-12);
end

% Lift axis completes the right-handed triad
eL = cross(eD, eY);
eL = eL / (norm(eL) + 1e-12);

L  = dot(Ftot, eL);
Y  = dot(Ftot, eY);

Di = dot(F_ind, eD);
Dp = dot(F_prof, eD);
D  = dot(Ftot, eD);

res = struct();
res.iter        = iter_used;
res.converged   = converged;
res.err         = err;
res.err_best    = err_best;
res.stagnated   = stagnated;
res.relax_final = relax_cur;
res.F           = Ftot;
res.M           = Mtot;
res.CL          = L  / (qinf * Sref);
res.CY          = Y  / (qinf * Sref);
res.CDi         = Di / (qinf * Sref);
res.CDp         = Dp / (qinf * Sref);
res.CD          = D  / (qinf * Sref);
res.Cm_y        = Mtot(2) / (qinf * Sref * cref);
res.Cl_x        = Mtot(1) / (qinf * Sref * bref);
res.Cn_z        = Mtot(3) / (qinf * Sref * bref);
res.alpha_deg    = alpha_deg;
res.beta_deg     = beta_deg;
res.camber_param = camber_param;
res.Sref         = Sref;
res.cref         = cref;
res.bref         = bref;
res.alpha_eff_deg   = alpha_eff_deg;
res.alpha_polar_deg = alpha_polar_deg;
res.Re_loc          = Re_loc;
res.Cl2d            = Cl2d;
res.Cd2d            = Cd2d;
res.Cm2d            = Cm2d;

% return gamma to warm-start next alpha
gamma_out = gamma;

end

% =====================================================================
% Strip helpers
% =====================================================================
function [strips, strip_shape] = build_strips_and_shape(panels, rings, idxSurf, gamma_lin)
%BUILD_STRIPS_AND_SHAPE  Build strip definitions for polar coupling.
%
% IMPORTANT for Nx>1:
%   - Strip spans the full chord, so store both LE and TE panel indices.
%   - Store an eval index near the 3/4-chord of the full chord.

strips = struct('idx',{},'le_idx',{},'te_idx',{},'eval_idx',{});
strip_shape = {};

surf_ids = unique([panels.surf]);
for s = surf_ids
    map = idxSurf{s};
    [Ny, ~] = size(map);
    for ii = 1:Ny
        row = map(ii,:);
        row = row(row>0);
        if isempty(row)
            continue;
        end

        le = row(1);
        te = row(end);

        nChord = numel(row);
        j_cand = (1:nChord);
        frac_cp = (j_cand - 0.25) ./ max(nChord,1);
        [~, j_eval] = min(abs(frac_cp - 0.75));
        eval_idx = row(j_eval);

        strips(end+1).idx    = row;
        strips(end).le_idx   = le;
        strips(end).te_idx   = te;
        strips(end).eval_idx = eval_idx;

        denom = gamma_lin(te);
        if abs(denom) < 1e-12
            sh = zeros(numel(row),1);
            sh(end) = 1;
        else
            sh = gamma_lin(row) / denom;
        end
        strip_shape{end+1} = sh;
    end
end

end

% =====================================================================
% Ring builder (chordwise-connected) + mapping
% =====================================================================
function [cp, nrm, rings, idxSurf] = build_connected_rings(panels, Vdir, Ltrail)

N = numel(panels);
cp  = zeros(N,3);
nrm = zeros(N,3);

qc_in  = zeros(N,3);
qc_out = zeros(N,3);

% per-surface map
surf_ids = unique([panels.surf]);
idxSurf = cell(1, max(surf_ids));

for s = surf_ids
    idx = find([panels.surf] == s);
    Ny = max([panels(idx).i]);
    Nx = max([panels(idx).j]);
    map = zeros(Ny, Nx);
    for k = idx(:).'
        map(panels(k).i, panels(k).j) = k;
    end
    idxSurf{s} = map;
end

% pass 1: collocation points, normals, quarter-chord points
for k = 1:N
    P = panels(k);
    p1=P.p1; p2=P.p2; p3=P.p3; p4=P.p4;

    c1 = p1 + 0.75*(p4 - p1);
    c2 = p2 + 0.75*(p3 - p2);
    cp(k,:) = 0.5*(c1 + c2);

    nn = cross(p2-p1, p4-p1);
    if norm(nn) < 1e-14
        nn = [0 0 1];
    end
    nn = nn / (norm(nn) + 1e-12);
    nrm(k,:) = nn;

    qc_in(k,:)  = p1 + 0.25*(p4 - p1);
    qc_out(k,:) = p2 + 0.25*(p3 - p2);
end

% pass 2: rings
rings = repmat(struct('p1',[],'p2',[],'p3',[],'p4',[], ...
                      'is_te',false,'wake_p3',[],'wake_p4',[], ...
                      'bound_mid',[],'bound_vec',[]), N, 1);

for k = 1:N
    P = panels(k);
    s  = P.surf;
    ii = P.i;
    jj = P.j;
    map = idxSurf{s};
    Nx = size(map,2);

    rings(k).p1 = qc_in(k,:);
    rings(k).p2 = qc_out(k,:);

    if jj < Nx
        k_dn = map(ii, jj+1);
        rings(k).p3 = qc_out(k_dn,:);
        rings(k).p4 = qc_in(k_dn,:);
        rings(k).is_te = false;
        rings(k).wake_p3 = [NaN NaN NaN];
        rings(k).wake_p4 = [NaN NaN NaN];
    else
        rings(k).p3 = P.p3;
        rings(k).p4 = P.p4;
        rings(k).is_te = true;
        rings(k).wake_p3 = rings(k).p3 + Ltrail * Vdir;
        rings(k).wake_p4 = rings(k).p4 + Ltrail * Vdir;
    end

    rings(k).bound_mid = 0.5*(rings(k).p1 + rings(k).p2);
    rings(k).bound_vec = rings(k).p2 - rings(k).p1;
end

end

% =====================================================================
% Induced velocity
% =====================================================================
function v = induced_velocity_ring(pt, ring)
v = [0 0 0];
v = v + vortex_segment_unit(pt, ring.p1, ring.p2);
v = v + vortex_segment_unit(pt, ring.p2, ring.p3);
v = v + vortex_segment_unit(pt, ring.p4, ring.p1);

if ring.is_te
    v = v + vortex_segment_unit(pt, ring.p3, ring.wake_p3);
    v = v + vortex_segment_unit(pt, ring.wake_p4, ring.p4);
else
    v = v + vortex_segment_unit(pt, ring.p3, ring.p4);
end
end

function v = vortex_segment_unit(p, a, b)
r1 = p - a;
r2 = p - b;
r0 = b - a;

cr = cross(r1, r2);
cr2 = dot(cr, cr);

if cr2 < 1e-12
    v = [0 0 0];
    return;
end

r1n = norm(r1);
r2n = norm(r2);
if r1n < 1e-12 || r2n < 1e-12
    v = [0 0 0];
    return;
end

term = dot(r0, (r1./r1n - r2./r2n));
v = (1/(4*pi)) * (cr / cr2) * term;
end

% =====================================================================
% Geometry helpers
% =====================================================================
function R = rot_y(theta)
R = [ cos(theta) 0 sin(theta);
      0          1 0;
     -sin(theta) 0 cos(theta)];
end

function P2 = rotate_panel_about_point(P, R, c)
P2 = P;
P2.p1 = (R*(P.p1 - c).').'+c;
P2.p2 = (R*(P.p2 - c).').'+c;
P2.p3 = (R*(P.p3 - c).').'+c;
P2.p4 = (R*(P.p4 - c).').'+c;
end

function S = estimate_total_area_from_panels(panels)
S = 0;
for k = 1:numel(panels)
    p1 = panels(k).p1; p2 = panels(k).p2; p3 = panels(k).p3; p4 = panels(k).p4;
    A1 = 0.5*norm(cross(p2-p1, p4-p1));
    A2 = 0.5*norm(cross(p3-p2, p4-p2));
    S = S + (A1 + A2);
end
end

function S = estimate_projected_area_xy_from_panels(panels)
S = 0;
for k = 1:numel(panels)
    p1 = panels(k).p1(1:2); p2 = panels(k).p2(1:2); p3 = panels(k).p3(1:2); p4 = panels(k).p4(1:2);

    v21 = p2 - p1;
    v41 = p4 - p1;
    A1 = 0.5 * abs(v21(1)*v41(2) - v21(2)*v41(1));

    v32 = p3 - p2;
    v42 = p4 - p2;
    A2 = 0.5 * abs(v32(1)*v42(2) - v32(2)*v42(1));

    S = S + (A1 + A2);
end
end

function bref = estimate_span_from_panels(panels)
%ESTIMATE_SPAN_FROM_PANELS  Estimate full span (tip-to-tip) from panel vertices.

ys = zeros(4*numel(panels),1);
for k = 1:numel(panels)
    P = panels(k);
    ys(4*(k-1)+1) = P.p1(2);
    ys(4*(k-1)+2) = P.p2(2);
    ys(4*(k-1)+3) = P.p3(2);
    ys(4*(k-1)+4) = P.p4(2);
end
bref = max(ys) - min(ys);
bref = max(bref, 1e-9);
end

function cref = estimate_mac_main_wing(wing)
%ESTIMATE_MAC_MAIN_WING  Numerical geometric MAC from actual chord distribution.
%
% For a symmetric wing:
%   S_half = ∫ c(y) dy
%   MAC    = (∫ c(y)^2 dy) / S_half

if ~isfield(wing,'surfaces') || isempty(wing.surfaces)
    cref = NaN;
    return;
end

S0 = wing.surfaces{1};
if ~isfield(S0,'nodes') || isempty(S0.nodes)
    cref = NaN;
    return;
end

nodes = S0.nodes;
Ny1 = size(nodes,1);

yq = zeros(Ny1,1);
c  = zeros(Ny1,1);
for i = 1:Ny1
    LE = squeeze(nodes(i,1,:)).';
    TE = squeeze(nodes(i,end,:)).';
    c(i) = norm(TE - LE);
    qc = LE + 0.25*(TE - LE);
    yq(i) = qc(2);
end

[yq, idx] = sort(yq);
c = c(idx);

S_half = trapz(yq, c);
I2     = trapz(yq, c.^2);

if S_half < 1e-12
    cref = NaN;
else
    cref = I2 / S_half;
end
end