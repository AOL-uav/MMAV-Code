# Cleanup notes

The earlier development folder contained several generations of scripts. The clean package intentionally removes the old exploratory/demo layer and keeps only the current workflow.

## Removed from main workflow

Simple/local aero development scripts:

- `demo_mmav_simple_panel_aero.m`
- `demo_mmav_alpha_trim_sweep.m`
- `demo_mmav_cg_tail_trim_map.m`
- `demo_mmav_tail_area_curl_map.m`
- `demo_mmav_candidate_trim_cases.m`
- `demo_mmav_trim_contour_candidates.m`
- `demo_mmav_constrained_trim_search.m`
- `demo_mmav_recommended_candidate_detail.m`
- `demo_mmav_candidate_speed_sweep.m`
- `demo_mmav_local_sensitivity.m`
- `mmav_assign_simple_panel_aero.m`
- `mmav_eval_simple_aero_state.m`
- `mmav_default_simple_aero_config.m`
- `mmav_passive_trim_metrics.m`

Debug/check scripts replaced by the clean runs:

- `demo_mmav_coarse_component_model.m`
- `demo_mmav_vlm_single_state.m`
- `demo_mmav_vlm_alpha_sweep.m`
- `demo_mmav_vlm_candidate_check.m`
- `demo_mmav_vlm_frame_corrected_cg_sweep.m`
- `demo_mmav_vlm_geometry_sanity.m`
- `demo_mmav_vlm_tail_gap_sign_diagnostic.m`
- `demo_mmav_vlm_safe_tail_cg_sweep.m`

Old or duplicate tail-design helpers:

- `mmav_apply_tail_design.m`
- old square-root span/chord scaling should not be used for VLM tail design because it can cause lateral wing/tail overlap.

## Kept functionality

- `RUN_01_component_model.m`: component/CG/panel geometry model.
- `RUN_02_trim_analysis.m`: one-case VLM passive-trim analysis.
- `RUN_03_tail_design_search.m`: free tail geometry sweep with trim/stability/L/D screening.

The clean package still includes the required low-level VLM and optimized-wing helpers under `external_opt0615/`.
