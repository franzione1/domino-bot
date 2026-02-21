# Tasks to reach 100% confidence for Domino pick-and-place

Follow and complete each task in order. Mark them off as you go.

[X] T01 - Fix gripper controller integration: implemented `simulate_gripper` parameter and fallback simulation when gripper controller/action is unavailable.

[ ] T02 - Verify and correct SRDF/URDF root mapping: ensure the virtual joint's child frame matches the URDF root or set virtual joint correctly in SRDF.

[ ] T03 - Add execution checks: modify `robot_mover.cpp` to plan first (getPlan), verify plan success, then execute only when plan valid; handle execution failures and retries.

[ ] T04 - Implement planning scene updates: add/maintain collision objects for dominoes and table in the PlanningScene so planners avoid collisions.

[ ] T05 - Implement attach/detach logic: after grasping, attach the object to the robot in the planning scene; detach after release.

[ ] T06 - Replace simple pose moves with Cartesian approach/retreat: use Cartesian waypoints for approach and retreat to improve grasp stability.

[ ] T07 - Add grasp candidate generator: compute multiple candidate grasps per domino (different lateral offsets and small yaw variations) and attempt until success.

[ ] T08 - Integrate robust gripper control with state feedback: use gripper status to confirm closed/open and detect failed grasps.

[ ] T09 - Tuning OMPL and controller parameters: adjust planners' time/attempts, tolerances, and controller execution parameters to reduce planning failures.

[ ] T10 - Add timeout and safe-stop behavior: ensure the robot stops and recovers safely on repeated failures or unexpected states.

[ ] T11 - Improve vision integration: make vision publish semantic labels (color, confidence, orientation) and include object bounding/orientation when available.

[ ] T12 - Calibrate vision → world transform: verify `pixel_to_real` mapping and camera extrinsics so target coordinates align precisely with robot frame.

[ ] T13 - Add verification step after place: re-scan placed domino to confirm color alignment; implement corrective rotation if necessary.

[ ] T14 - Add logging and metrics: track plan success rate, execution success, grasp success, and failures to guide tuning.

[ ] T15 - Create automated integration tests: use the test publisher + headless Gazebo runs to run deterministic pick/place sequences and assert final states.

[ ] T16 - Add parameterized YAML configs: expose domino size, gap, approach distances, HSV thresholds, and planner tolerances as editable parameters.

[ ] T17 - Continuous integration (optional): add CI job that builds and runs headless integration tests to prevent regressions.

[ ] T18 - Safety review & limits: verify joint limits, workspace bounds, and emergency stop integration for safe physical robot deployment (if moving to hardware).

[ ] T19 - Documentation & runbook: write a short README with run commands, tuning tips, and failure diagnostics for operators.

[ ] T20 - Final validation: run multiple randomized scenarios (different domino arrangements/colors) with the full stack and achieve >99% success rate on pick+place and color alignment.

---
Notes:
- Prioritize T01-T06 first; they are the most likely to fix current run failures.
- T07-T09 improve robustness; T11-T13 ensure semantic correctness (color matching).
- CI, docs, and safety are last but required before trusting the system in production.
