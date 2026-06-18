function wing = wing_geometry_build_flat_wing(p)
%WING_GEOMETRY_BUILD_FLAT_WING  Build a flat wing geometry struct.
%
% This is a thin wrapper around the existing geometry generator used by the
% original demo:
%   build_flat_wing_with_optional_winglet.m
%
% Keeping this as a dedicated function makes it easy to:
%   - unit-test geometry generation
%   - swap geometry generators later without touching the aero solver
%
% Inputs
%   p : struct of planform/mesh/winglet parameters (same as original demo)
%
% Output
%   wing : wing struct containing surfaces/panels/nodes

if nargin < 1
    error('wing_geometry_build_flat_wing requires an input parameter struct p.');
end

if exist('build_flat_wing_with_optional_winglet','file') ~= 2
    error([ ...
        'Missing dependency: build_flat_wing_with_optional_winglet.m\n', ...
        'Make sure it is on the MATLAB path before running this demo.']);
end

wing = build_flat_wing_with_optional_winglet(p);

if ~isstruct(wing) || ~isfield(wing,'surfaces')
    error('Geometry builder returned an unexpected wing struct.');
end

end
