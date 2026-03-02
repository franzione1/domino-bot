Domino Project - Runbook

- Build:

```bash
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
```

- Launch full stack (MoveIt + Gazebo):

```bash
ros2 launch domino_project final_system_moveit.launch.py
```

- Run headless integration test (simple):

```bash
./src/domino_project/tests/integration/test_pick_place_integration.sh
```

- Run randomized validation (assumes system launched separately):

```bash
python3 src/domino_project/scripts/run_randomized_validation.py
```

- Basic safety check (URDF joint limits):

```bash
python3 src/domino_project/scripts/safety_check.py
```

- Tuning: edit `src/domino_project/config/params.yaml` and `src/domino_project/config/ompl_tuning.yaml` then rebuild.
