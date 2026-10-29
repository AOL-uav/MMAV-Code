function ref = mmav_opt0615_layout_reference()
%MMAV_OPT0615_LAYOUT_REFERENCE  Frame conversion constants from opt_0615/cg.m.
%
% The opt_0615/cg.m layout script uses a global chordwise coordinate whose
% origin is the *unshifted original root leading edge* and whose +x direction
% is aft.  The coarse MMAV scripts use body/build frame B whose origin is the
% fuselage nose centerline, also with +x aft.
%
% Therefore:
%   x_B = x_opt0615_global - x_fuselage_nose_global
%
% These numbers are the selected SM_min = 0.06 case reconstructed from the
% uploaded opt_0615/cg.m configuration.  If cg.m is edited later, update this
% file or replace these constants by a direct call to the layout optimizer.

ref = struct();

% Fuselage geometry in opt_0615/cg.m global coordinate.
ref.fuselage_x_center_global_mm = 55.0;
ref.fuselage_length_x_mm        = 70.0;
ref.fuselage_nose_global_mm     = ref.fuselage_x_center_global_mm - 0.5*ref.fuselage_length_x_mm;
ref.fuselage_tail_global_mm     = ref.fuselage_x_center_global_mm + 0.5*ref.fuselage_length_x_mm;

% Selected wing/CG layout from the SM_min = 0.06 case.
% In opt_0615/cg.m this is called the whole-wing x-offset.
ref.selected_wing_offset_global_mm = 27.77;
ref.selected_cg_global_mm          = 55.52;

% Body-frame values used by the coarse component/VLM scripts.
ref.wing_rootLE_B_mm = ref.selected_wing_offset_global_mm - ref.fuselage_nose_global_mm;
ref.cg_B_mm          = ref.selected_cg_global_mm          - ref.fuselage_nose_global_mm;

% Useful local wing coordinate.
ref.cg_local_from_shifted_rootLE_mm = ref.selected_cg_global_mm - ref.selected_wing_offset_global_mm;

% Approximate references from opt_0615 optimized wing.
ref.MAC_mm = 156.62;
ref.cg_local_over_MAC = ref.cg_local_from_shifted_rootLE_mm / ref.MAC_mm;

% Suggested VLM sweep ranges in body frame B, not opt_0615 global frame.
ref.suggested_cg_sweep_B_mm = 28:2:50;
ref.suggested_tail_scales   = [0 0.5 1 1.5 2 2.5 3];
ref.suggested_curl_deg      = [20 35 55];
end
