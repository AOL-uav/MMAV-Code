function wing = build_flat_wing_with_optional_winglet(p)
%BUILD_FLAT_WING_WITH_OPTIONAL_WINGLET  Flat wing geometry builder (L/R mirrored)
%
% This generates a *flat* (zero-camber, zero-thickness) lifting surface mesh
% suitable for the inviscid VLM in demo_flat_wing_vlm.m.
%
% --------------------------- REQUIRED (main wing) ------------------------
%   p.b, p.c_root, p.c_tip, p.sweepLE_deg, p.dihedral_deg
%   p.Ny_half, p.Nx
%
% --------------------------- PLANFORM OPTIONS ----------------------------
% Default planform uses straight LE sweep + linear taper:
%   xLE(y) = tan(sweepLE)*y,  c(y) linear => xTE = xLE + c
%
% You can override planform with LE/TE curves vs normalized span eta in [0..1]
% (eta=0 root, eta=1 tip of a half-wing):
%   p.plan_eta    : vector (strictly increasing) in [0..1]
%   p.plan_xLE    : LE x [m] at p.plan_eta (optional)
%   p.plan_xTE    : TE x [m] at p.plan_eta (optional)
%   p.plan_c      : chord [m] at p.plan_eta (optional, used if plan_xTE absent)
%   p.plan_interp : 'pchip' (default) | 'linear' | 'makima' | 'spline'
%
% For even more flexibility (piecewise kinks / discontinuities / segments),
% you can split the half-wing into multiple *surfaces*:
%   p.plan_segments(k) with fields:
%     .eta0, .eta1          : span range of this segment in [0..1]
%     .Ny                   : optional spanwise panels for this segment
%     .plan_eta/.plan_xLE/.plan_xTE/.plan_c/.plan_interp (segment overrides)
%     .twist_eta/.twist_deg/.twist_interp               (segment overrides)
%
% Notes:
%  - Kinks (continuous but non-smooth) are fine with plan_interp='linear'.
%  - True discontinuities (jump in LE/TE) are only possible via segments.
%  - If you specify both plan_xTE and plan_c, plan_xTE wins.
%
% --------------------------- DIHEDRAL / TWIST ----------------------------
% Dihedral default: constant p.dihedral_deg, z(y)=tan(dihedral)*y
%
% Optional *polyhedral* dihedral (varies with span):
%   p.dihedral_eta       : vector in [0..1]
%   p.dihedral_deg_table : dihedral angle [deg] at those eta
%   p.dihedral_interp    : interp method (default 'pchip')
% z(y) is formed by integrating dz/dy = tan(dihedral(y)).
%
% Optional twist (washout / incidence) distribution:
%   p.twist_eta    : vector in [0..1]
%   p.twist_deg    : twist angle [deg] at those eta
%   p.twist_interp : interp method (default 'pchip')
% Twist is applied as a rotation about the local quarter-chord line.
% Sign convention: positive twist rotates nose-down (reduces local AoA),
% matching rot_y() used elsewhere.
%
% --------------------------- OPTIONAL WINGLET ----------------------------
%   p.winglet_height, p.winglet_cant_deg, p.winglet_sweepLE_deg, p.winglet_chord_ratio
%
% Output:
%   wing.surfaces{k}.nodes   : (Ny+1)x(Nx+1)x3
%   wing.surfaces{k}.panels  : struct array with p1..p4
%   wing.surfaces{k}.planform: (main surfaces only) debug info

surfaces = {};

% --- Main right half-wing surfaces (may be multiple segments) ---
S_main_R_list = make_main_half_surfaces(p);
for i = 1:numel(S_main_R_list)
    surfaces{end+1} = S_main_R_list{i}; %#ok<AGROW>
end

% --- Main left half-wing (mirror y) ---
for i = 1:numel(S_main_R_list)
    surfaces{end+1} = mirror_surface_y(S_main_R_list{i}); %#ok<AGROW>
end

% --- Optional winglet attached to right *tip* of the last segment ---
if isfield(p,'winglet_height') && p.winglet_height > 0
    S_tip = S_main_R_list{end};
    tip_nodes = squeeze(S_tip.nodes(end, :, :)); % (Nx+1) x 3
    tip_LE = tip_nodes(1, :);
    tip_TE = tip_nodes(end, :);

    c_tip = norm(tip_TE - tip_LE);
    c_w = p.winglet_chord_ratio * c_tip;

    S_wR = make_winglet_surface( ...
        tip_LE, c_w, p.winglet_height, ...
        p.winglet_sweepLE_deg, p.winglet_cant_deg, ...
        p.Ny_half, p.Nx);

    surfaces{end+1} = S_wR;
    surfaces{end+1} = mirror_surface_y(S_wR);
end

wing = struct();
wing.surfaces = surfaces;

end

% =====================================================================
% Main half-wing (right side): 1 or more spanwise segments
% =====================================================================
function S_list = make_main_half_surfaces(p)

halfspan = p.b/2;
Ny_total = p.Ny_half;
Nx = p.Nx;

% --- Build a z(y) mapping (supports polyhedral dihedral) ---
z_of_y = build_z_of_y(p, halfspan);

% --- Segment definition ---
if isfield(p,'plan_segments') && ~isempty(p.plan_segments)
    segs = p.plan_segments;
    if ~isstruct(segs)
        error('p.plan_segments must be a struct array.');
    end
    % Basic checks
    for k = 1:numel(segs)
        if ~isfield(segs(k),'eta0') || ~isfield(segs(k),'eta1')
            error('Each p.plan_segments(k) must contain fields eta0 and eta1.');
        end
        if segs(k).eta1 <= segs(k).eta0
            error('Segment %d has eta1<=eta0.', k);
        end
    end
else
    segs = struct('eta0',0,'eta1',1);
end

% --- Allocate spanwise panels to segments if not provided ---
Ny_seg = zeros(1, numel(segs));
provided = false(1, numel(segs));
for k = 1:numel(segs)
    if isfield(segs(k),'Ny') && ~isempty(segs(k).Ny)
        Ny_seg(k) = max(1, round(segs(k).Ny));
        provided(k) = true;
    end
end
if ~any(provided)
    lens = zeros(1, numel(segs));
    for k = 1:numel(segs)
        lens(k) = segs(k).eta1 - segs(k).eta0;
    end
    lens = lens / max(sum(lens), 1e-12);
    Ny_seg = max(1, round(Ny_total * lens));
    % Fix rounding so sum equals Ny_total
    d = Ny_total - sum(Ny_seg);
    if d ~= 0
        Ny_seg(end) = max(1, Ny_seg(end) + d);
    end
else
    % If user provides some Ny, keep them and allocate remaining proportionally
    remaining = Ny_total - sum(Ny_seg);
    if remaining < 0
        warning('Sum of provided segment Ny (%d) exceeds p.Ny_half (%d). Using provided values.', sum(Ny_seg), Ny_total);
    elseif remaining > 0
        lens = zeros(1, numel(segs));
        for k = 1:numel(segs)
            if ~provided(k)
                lens(k) = segs(k).eta1 - segs(k).eta0;
            end
        end
        if sum(lens) < 1e-12
            % If all missing segments have zero length (shouldn't happen), dump remainder to last
            Ny_seg(end) = Ny_seg(end) + remaining;
        else
            lens = lens / sum(lens);
            add = round(remaining * lens);
            Ny_seg = Ny_seg + add;
            d = Ny_total - sum(Ny_seg);
            if d ~= 0
                Ny_seg(end) = max(1, Ny_seg(end) + d);
            end
        end
    end
end

% --- Build surfaces ---
S_list = cell(1, numel(segs));
for k = 1:numel(segs)
    S_list{k} = make_main_half_surface_segment(p, segs(k), Ny_seg(k), Nx, halfspan, z_of_y);
    S_list{k}.name = sprintf('main_half_seg%d', k);
end

end

function S = make_main_half_surface_segment(p, seg, Ny, Nx, halfspan, z_of_y)

% Span stations for this segment
eta0 = seg.eta0;
eta1 = seg.eta1;

y0 = eta0 * halfspan;
y1 = eta1 * halfspan;

y = linspace(y0, y1, Ny+1).';
eta = y / max(halfspan, 1e-12);

% Defaults: straight LE sweep + linear chord
sweep = deg2rad(p.sweepLE_deg);
xLE_default = tan(sweep) * y;

c_default   = p.c_root + (p.c_tip - p.c_root) * (y / max(halfspan, 1e-12));
xTE_default = xLE_default + c_default;

% Planform overrides (segment overrides p)
xLE = eval_plan_quantity('xLE', xLE_default, eta, y, halfspan, p, seg);

% Prefer explicit xTE; else chord; else default
xTE = eval_plan_quantity('xTE', xTE_default, eta, y, halfspan, p, seg);

% If plan_xTE not provided anywhere but chord was provided, use it
has_xTE = plan_field_exists(p, seg, 'plan_xTE') || plan_fun_exists(p, seg, 'plan_xTE_fun');
if ~has_xTE
    c_override = eval_plan_quantity('c', (xTE_default - xLE_default), eta, y, halfspan, p, seg);
    xTE = xLE + c_override;
end

% Ensure positive chord
c = xTE - xLE;
c = max(c, 1e-4);
xTE = xLE + c;

% Dihedral / polyhedral z(y)
z = z_of_y(y);

% Twist distribution (segment overrides p)
twist_deg = eval_twist_deg(eta, p, seg);

% Build nodes: chordwise straight lines from LE to TE
xi = linspace(0, 1, Nx+1);

nodes = zeros(Ny+1, Nx+1, 3);
for i = 1:Ny+1
    for j = 1:Nx+1
        x = xLE(i) + xi(j) * (xTE(i) - xLE(i));
        nodes(i,j,:) = [x, y(i), z(i)];
    end
end

% Apply twist about local quarter-chord line
if any(abs(twist_deg) > 1e-12)
    nodes = apply_twist_about_quarter_chord(nodes, xLE, xTE, y, z, twist_deg);
end

S = struct();
S.nodes = nodes;
S.panels = surface_nodes_to_panels(nodes);

% Attach planform debug info for the *main* surfaces
S.planform = struct();
S.planform.y = y;
S.planform.eta = eta;
S.planform.xLE = xLE;
S.planform.xTE = xTE;
S.planform.c = c;
S.planform.z = z;
S.planform.twist_deg = twist_deg;
S.planform.eta_range = [eta0 eta1];

end

% =====================================================================
% Planform evaluation helpers
% =====================================================================
function v = eval_plan_quantity(kind, v_default, eta, y, halfspan, p, seg)
% kind in {'xLE','xTE','c'}

% 1) function-handle override (segment then global)
fun_field = ['plan_' kind '_fun'];
f = [];
if isfield(seg, fun_field) && ~isempty(seg.(fun_field))
    f = seg.(fun_field);
elseif isfield(p, fun_field) && ~isempty(p.(fun_field))
    f = p.(fun_field);
end
if ~isempty(f)
    v = call_span_fun(f, eta, y, halfspan, p, seg);
    v = v(:);
    if numel(v) ~= numel(eta)
        error('%s must return a vector with same length as eta.', fun_field);
    end
    return;
end

% 2) table override via interp1
eta_field = 'plan_eta';
val_field = ['plan_' kind];
if plan_field_exists(p, seg, val_field) && plan_field_exists(p, seg, eta_field)
    [ek, vk, method] = get_plan_table(p, seg, eta_field, val_field);
    v = interp1(ek, vk, eta, method, 'extrap');
    return;
end

% 3) fallback
v = v_default;
end

function tf = plan_field_exists(p, seg, field)
if isfield(seg, field) && ~isempty(seg.(field))
    tf = true; return;
end
if isfield(p, field) && ~isempty(p.(field))
    tf = true; return;
end
tf = false;
end

function tf = plan_fun_exists(p, seg, field)
if isfield(seg, field) && ~isempty(seg.(field))
    tf = true; return;
end
if isfield(p, field) && ~isempty(p.(field))
    tf = true; return;
end
tf = false;
end

function [ek, vk, method] = get_plan_table(p, seg, eta_field, val_field)
% Pull plan_eta and plan_val from seg if present else p.

if isfield(seg, eta_field) && ~isempty(seg.(eta_field))
    ek = seg.(eta_field)(:);
else
    ek = p.(eta_field)(:);
end

if isfield(seg, val_field) && ~isempty(seg.(val_field))
    vk = seg.(val_field)(:);
else
    vk = p.(val_field)(:);
end

if numel(ek) ~= numel(vk)
    error('%s must match length of %s.', val_field, eta_field);
end

% Allow local-eta tables per segment
eta_is_local = false;
if isfield(seg, 'plan_eta_is_local') && ~isempty(seg.plan_eta_is_local)
    eta_is_local = logical(seg.plan_eta_is_local);
end
if eta_is_local
    ek = seg.eta0 + ek * (seg.eta1 - seg.eta0);
end

% Checks
if any(~isfinite(ek)) || any(~isfinite(vk))
    error('%s/%s contains NaN/Inf.', eta_field, val_field);
end
if any(ek < -1e-9) || any(ek > 1+1e-9)
    error('%s must lie within [0..1] (global normalized span).', eta_field);
end

% Sort
[ek, idx] = sort(ek);
vk = vk(idx);

if numel(unique(ek)) ~= numel(ek)
    error('%s has duplicate entries. For discontinuities split into segments.', eta_field);
end

% Interp method
method = 'pchip';
if isfield(seg,'plan_interp') && ~isempty(seg.plan_interp)
    method = seg.plan_interp;
elseif isfield(p,'plan_interp') && ~isempty(p.plan_interp)
    method = p.plan_interp;
end

end

function v = call_span_fun(f, eta, y, halfspan, p, seg)
% Call function handle with a flexible signature.
% Recommended signatures:
%   v = f(eta)
% or
%   v = f(eta, y, halfspan, p, seg)

try
    n = nargin(f);
catch
    n = 1;
end

if n <= 1
    v = f(eta);
else
    v = f(eta, y, halfspan, p, seg);
end
end

% =====================================================================
% Dihedral support
% =====================================================================
function z_of_y = build_z_of_y(p, halfspan)

% Option A: direct z(eta) table
if isfield(p,'z_eta') && isfield(p,'z_table') && ~isempty(p.z_eta) && ~isempty(p.z_table)
    ek = p.z_eta(:);
    vk = p.z_table(:);
    [ek, idx] = sort(ek);
    vk = vk(idx);
    if numel(ek) ~= numel(vk)
        error('p.z_eta must match length of p.z_table.');
    end
    method = 'pchip';
    if isfield(p,'z_interp') && ~isempty(p.z_interp)
        method = p.z_interp;
    end
    z_of_y = @(yq) interp1(ek*halfspan, vk, yq, method, 'extrap');
    return;
end

% Option B: polyhedral dihedral angle table => integrate to z(y)
if isfield(p,'dihedral_eta') && isfield(p,'dihedral_deg_table') && ~isempty(p.dihedral_eta) && ~isempty(p.dihedral_deg_table)
    ek = p.dihedral_eta(:);
    dk = p.dihedral_deg_table(:);
    if numel(ek) ~= numel(dk)
        error('p.dihedral_eta must match length of p.dihedral_deg_table.');
    end
    [ek, idx] = sort(ek);
    dk = dk(idx);

    method = 'pchip';
    if isfield(p,'dihedral_interp') && ~isempty(p.dihedral_interp)
        method = p.dihedral_interp;
    end

    y_fine = linspace(0, halfspan, 2001).';
    eta_fine = y_fine / max(halfspan, 1e-12);
    d_deg = interp1(ek, dk, eta_fine, method, 'extrap');
    z_fine = cumtrapz(y_fine, tan(deg2rad(d_deg)));

    z_of_y = @(yq) interp1(y_fine, z_fine, yq, 'pchip', 'extrap');
    return;
end

% Option C: constant dihedral
if ~isfield(p,'dihedral_deg')
    error('p.dihedral_deg is required when no polyhedral z() table is provided.');
end

dihedral = deg2rad(p.dihedral_deg);
z_of_y = @(yq) tan(dihedral) * yq;

end

% =====================================================================
% Twist support
% =====================================================================
function twist_deg = eval_twist_deg(eta, p, seg)

% Function handle override (segment then global)
f = [];
if isfield(seg,'twist_fun') && ~isempty(seg.twist_fun)
    f = seg.twist_fun;
elseif isfield(p,'twist_fun') && ~isempty(p.twist_fun)
    f = p.twist_fun;
end
if ~isempty(f)
    twist_deg = call_span_fun(f, eta, [], [], p, seg);
    twist_deg = twist_deg(:);
    if numel(twist_deg) ~= numel(eta)
        error('twist_fun must return vector same length as eta.');
    end
    return;
end

% Table override
if isfield(seg,'twist_eta') && ~isempty(seg.twist_eta) && isfield(seg,'twist_deg') && ~isempty(seg.twist_deg)
    ek = seg.twist_eta(:);
    vk = seg.twist_deg(:);
    eta_is_local = false;
    if isfield(seg,'twist_eta_is_local') && ~isempty(seg.twist_eta_is_local)
        eta_is_local = logical(seg.twist_eta_is_local);
    end
    if eta_is_local
        ek = seg.eta0 + ek * (seg.eta1 - seg.eta0);
    end
    method = 'pchip';
    if isfield(seg,'twist_interp') && ~isempty(seg.twist_interp)
        method = seg.twist_interp;
    end
    twist_deg = interp1(ek, vk, eta, method, 'extrap');
    return;
end

if isfield(p,'twist_eta') && ~isempty(p.twist_eta) && isfield(p,'twist_deg') && ~isempty(p.twist_deg)
    ek = p.twist_eta(:);
    vk = p.twist_deg(:);
    method = 'pchip';
    if isfield(p,'twist_interp') && ~isempty(p.twist_interp)
        method = p.twist_interp;
    end
    twist_deg = interp1(ek, vk, eta, method, 'extrap');
    return;
end

% Default: no twist
% (vector length = numel(eta))
twist_deg = zeros(size(eta));

end

function nodes = apply_twist_about_quarter_chord(nodes, xLE, xTE, y, z, twist_deg)
% Rotate chordwise rows about the local quarter-chord line.

[Ny1, Nx1, ~] = size(nodes);

% Quarter-chord x location
c = xTE - xLE;
xQC = xLE + 0.25*c;

% Local axis tangent along the quarter-chord line
% Include sweep (dxQC/dy) and dihedral (dz/dy)
dxdy = gradient(xQC, y);
dzdy = gradient(z, y);

for i = 1:Ny1
    axis = [dxdy(i), 1.0, dzdy(i)];
    axis = axis / (norm(axis) + 1e-12);
    p0 = [xQC(i), y(i), z(i)];
    theta = deg2rad(twist_deg(i));

    for j = 1:Nx1
        pnt = squeeze(nodes(i,j,:)).';
        pnt2 = rotate_about_axis(pnt, p0, axis, theta);
        nodes(i,j,:) = reshape(pnt2, 1, 1, 3);
    end
end

end

function p2 = rotate_about_axis(p, p0, axis_unit, theta)
% Rodrigues rotation of point p about axis passing through p0.

k = axis_unit / (norm(axis_unit) + 1e-12);
v = p - p0;

v2 = v*cos(theta) + cross(k, v)*sin(theta) + k*dot(k, v)*(1 - cos(theta));
p2 = p0 + v2;
end

% =====================================================================
% Winglet surface
% =====================================================================
function S = make_winglet_surface(root_LE, chord, height, sweepLE_deg, cant_deg, Nh, Nx)

sweep = deg2rad(sweepLE_deg);
cant  = deg2rad(cant_deg);

% Winglet span direction (unit) in yz plane
d = [0, sin(cant), cos(cant)];   % vertical when cant=0

w  = linspace(0, height, Nh+1).';
xi = linspace(0, 1, Nx+1);

nodes = zeros(Nh+1, Nx+1, 3);
for i = 1:Nh+1
    xLE = root_LE(1) + tan(sweep) * w(i);
    base = root_LE + w(i) * d;
    for j = 1:Nx+1
        nodes(i,j,:) = [xLE + xi(j)*chord, base(2), base(3)];
    end
end

S = struct();
S.name  = 'winglet_half';
S.nodes = nodes;
S.panels = surface_nodes_to_panels(nodes);

end

% =====================================================================
% Panelization + mirroring
% =====================================================================
% function panels = surface_nodes_to_panels(nodes)
% % p1(inboard, LE), p2(outboard, LE), p3(outboard, TE), p4(inboard, TE)
% 
% [Ns, Nc, ~] = size(nodes);
% Np = (Ns-1)*(Nc-1);
% 
% panels = repmat(struct('p1',[],'p2',[],'p3',[],'p4',[]), Np, 1);
% 
% idx = 0;
% for i = 1:Ns-1
%     for j = 1:Nc-1
%         idx = idx + 1;
%         p1 = squeeze(nodes(i,   j,   :)).';
%         p2 = squeeze(nodes(i+1, j,   :)).';
%         p3 = squeeze(nodes(i+1, j+1, :)).';
%         p4 = squeeze(nodes(i,   j+1, :)).';
%         panels(idx).p1 = p1;
%         panels(idx).p2 = p2;
%         panels(idx).p3 = p3;
%         panels(idx).p4 = p4;
%     end
% end
% 
% end
function panels = surface_nodes_to_panels(nodes)
% p1(inboard, LE), p2(outboard, LE), p3(outboard, TE), p4(inboard, TE)

[Ns, Nc, ~] = size(nodes);
Np = (Ns-1)*(Nc-1);

panels = repmat(struct( ...
    'p1',[],'p2',[],'p3',[],'p4',[], ...
    'i',[],'j',[],'is_te',[]), Np, 1);

idx = 0;
for i = 1:Ns-1
    for j = 1:Nc-1
        idx = idx + 1;
        p1 = squeeze(nodes(i,   j,   :)).';
        p2 = squeeze(nodes(i+1, j,   :)).';
        p3 = squeeze(nodes(i+1, j+1, :)).';
        p4 = squeeze(nodes(i,   j+1, :)).';
        
        panels(idx).p1 = p1;
        panels(idx).p2 = p2;
        panels(idx).p3 = p3;
        panels(idx).p4 = p4;
        panels(idx).i  = i;                % spanwise panel index
        panels(idx).j  = j;                % chordwise panel index
        panels(idx).is_te = (j == (Nc-1)); % trailing edge panels
    end
end
end


function S2 = mirror_surface_y(S)
S2 = S;
S2.name = [S.name '_mirrored'];

nodes = S.nodes;
nodes(:,:,2) = -nodes(:,:,2);
nodes = flip(nodes,1);  

S2.nodes  = nodes;
S2.panels = surface_nodes_to_panels(nodes);

% Mirror planform debug if present (keep ordering consistent with nodes)
if isfield(S,'planform') && ~isempty(S.planform)
    S2.planform = S.planform;
    S2.planform.y        = flip(-S.planform.y);
    S2.planform.eta      = flip(S.planform.eta);
    S2.planform.xLE      = flip(S.planform.xLE);
    S2.planform.xTE      = flip(S.planform.xTE);
    S2.planform.c        = flip(S.planform.c);
    S2.planform.z        = flip(S.planform.z);
    S2.planform.twist_deg= flip(S.planform.twist_deg);
end
end
