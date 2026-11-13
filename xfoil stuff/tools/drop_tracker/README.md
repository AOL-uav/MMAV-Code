# Drop tracker

Single-file, offline tool: measure a glider drop from a side-on phone video. Open `drop_tracker.html` in any modern browser. No install, no network; the video never leaves the browser.

Workflow (also shown on the page):

1. Load the video, set its frame rate (30 or 60).
2. Set scale: click both ends of a known reference, enter its length in m. Optional: set a level line (horizon or ground) to remove camera roll.
3. Track: step to the release frame and click the glider; the video advances N frames after each click (default 2).
4. Select points and tag release, lowest, level-off, glide start, glide end. Release defaults to the first point, lowest to the deepest.
5. Read swoop depth, times, glide distance, height lost, glide ratio and angle, mean speed.
6. Fill in drop #, article, CG, bend, height, outcome, note; download the points CSV and the one-row summary CSV (columns match the drop log: drop #, article, CG, bend, height, outcome, swoop depth, glide dist, clip, note).

Keys: `,` and `.` step one frame, Enter or `N` steps N frames, Space plays/pauses, `Z` undoes the last point.

Limitations:

- Scale is valid only in the plane of the reference. Keep the reference at the same distance from the camera as the flight path, with the camera side-on. No other perspective correction.
- Heading and path straightness cannot be measured from one side camera.
- Browser frame stepping (`currentTime += 1/fps`) is not frame-exact; check the time shown. Variable-frame-rate phone video can drift.
- Distances are horizontal along the camera axis, after roll correction.
- Points CSV import restores points and tags only; the scale is not in the CSV (it is in the autosave for the same file).
- Autosave uses localStorage, keyed by video file name and size.

Self-test: open `drop_tracker.html?selftest` and read the PASS/FAIL line at the top. `?demo` loads a synthetic path to preview the layout.
