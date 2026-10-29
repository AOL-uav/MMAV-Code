function coeff = mmav_wind_axis_coefficients(out, state)
%MMAV_WIND_AXIS_COEFFICIENTS Compute body- and wind-axis coefficients.
%
% coeff = mmav_wind_axis_coefficients(out, state)
%
% Body frame B:
%   x_B aft, y_B right, z_B up
%
% Important distinction:
%   C_Fx is a body-axis force coefficient. It can become negative at high
%   alpha because lift has a forward body-x component.
%
%   C_D_wind is the true drag coefficient projected along the drag/relative
%   wind direction eD_B. This should remain positive for passive aero loads.

axesB = mmav_wind_axes_B(state);
F = out.loads.F_B_N(:).';
M = out.loads.M_CG_B_Nm(:).';
qS = max(out.q_Pa*out.Sref_m2, 1e-12);
qSc = max(out.q_Pa*out.Sref_m2*out.cref_m, 1e-12);

coeff = struct();
coeff.eD_B = axesB.eD_B;
coeff.eL_B = axesB.eL_B;
coeff.CF_B = F/qS;
coeff.CM_B = M/qSc;
coeff.CFx_body = coeff.CF_B(1);
coeff.CFy_body = coeff.CF_B(2);
coeff.CFz_body = coeff.CF_B(3);
coeff.CD_wind = dot(F, axesB.eD_B)/qS;
coeff.CL_wind = dot(F, axesB.eL_B)/qS;
coeff.Cm_y = coeff.CM_B(2);
end
