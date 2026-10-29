function mmav_plot_coarse_vehicle(vehicle, panels)
%MMAV_PLOT_COARSE_VEHICLE Plot panelized geometry, coarse mass CGs, active CG.

if nargin < 2 || isempty(panels)
    panels = vehicle.panels;
end

figure('Name','MMAV coarse component model');
hold on; grid on; axis equal;

% Plot panels.
for k = 1:numel(panels)
    V = [panels(k).p1_B_m; panels(k).p2_B_m; panels(k).p3_B_m; panels(k).p4_B_m] * 1e3;
    if strcmpi(panels(k).component, 'wing')
        fc = [0.65 0.80 1.00];
    elseif strcmpi(panels(k).component, 'curled_tail')
        fc = [1.00 0.75 0.55];
    else
        fc = [0.85 0.85 0.85];
    end
    patch('Vertices', V, 'Faces', [1 2 3 4], ...
        'FaceColor', fc, 'FaceAlpha', 0.45, 'EdgeColor', [0.25 0.25 0.25]);
end

% Plot active CG.
rCG = vehicle.cg.r_active_B_m * 1e3;
plot3(rCG(1), rCG(2), rCG(3), 'kp', 'MarkerSize', 15, 'MarkerFaceColor', 'y');
text(rCG(1), rCG(2), rCG(3)+8, ' active CG', 'FontWeight','bold');

% Plot component CGs.
for k = 1:numel(vehicle.mass_components)
    rc = vehicle.mass_components(k).r_cg_B_m * 1e3;
    plot3(rc(1), rc(2), rc(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    text(rc(1), rc(2), rc(3)-8, [' ' vehicle.mass_components(k).name], 'Interpreter','none');
end

% Plot fuselage drag CP.
for k = 1:numel(vehicle.aero_bodies)
    rb = vehicle.aero_bodies(k).r_ref_B_m * 1e3;
    plot3(rb(1), rb(2), rb(3), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    text(rb(1), rb(2), rb(3)+5, [' ' vehicle.aero_bodies(k).name], 'Interpreter','none');
end

xlabel('x_B aft [mm]');
ylabel('y_B right [mm]');
zlabel('z_B up [mm]');
title('Coarse mass components + panelized aero surfaces');
view(35, 20);

% Draw body axes at origin.
L = 80;
quiver3(0,0,0,L,0,0,'k','LineWidth',1.4,'MaxHeadSize',0.6);
quiver3(0,0,0,0,L,0,'k','LineWidth',1.4,'MaxHeadSize',0.6);
quiver3(0,0,0,0,0,L,'k','LineWidth',1.4,'MaxHeadSize',0.6);
text(L,0,0,' x_B'); text(0,L,0,' y_B'); text(0,0,L,' z_B');
end
