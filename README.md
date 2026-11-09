# ros2-excavator

ROS 2 (Lyrical) model of a tracked excavator, viewable in RViz and driven with joint sliders.

The model is built from primitive shapes in xacro — no mesh files. The hydraulic cylinders
extend and retract with the joints.

## Packages

| Package | Contents |
|---|---|
| `excavator_description` | xacro model, launch file, RViz config |
| `excavator_gui` | Qt slider window; one slider per joint, read from the URDF |
| `excavator_hydraulics` | Node that publishes TF for the cylinder barrels and rods |

## Joints

| Joint | Moves | Range (rad) |
|---|---|---|
| `swing_joint` | upper body on the tracks | continuous (no limit; slider covers −π … π) |
| `boom_joint` | boom | −0.6 … 1.1 |
| `arm_joint` | arm (stick) | −2.6 … 0.3 |
| `bucket_joint` | bucket | −2.2 … 1.0 |

Positive angles raise the boom, arm and bucket.

## Requirements

ROS 2 Lyrical with `xacro`, `robot_state_publisher`, `rviz2`, `tf2_ros`, `python_qt_binding`
and `joint_state_publisher_gui` (used by `gui:=jsp`). `rosdep` installs them all:

```bash
rosdep install --from-paths src --ignore-src -y
```

or with apt:

```bash
sudo apt install ros-lyrical-xacro ros-lyrical-joint-state-publisher-gui
```

## Build and run

```bash
cd ~/excavator-workspace
source /opt/ros/lyrical/setup.bash
colcon build
source install/setup.bash
ros2 launch excavator_description display.launch.py
```

Launch arguments:

| Argument | Values | Default | |
|---|---|---|---|
| `gui` | `slider`, `jsp`, `none` | `slider` | `jsp` = `joint_state_publisher_gui`; `none` = joint states come from elsewhere |
| `rviz` | `true`, `false` | `true` | start RViz |
| `rviz_hidpi_fix` | `true`, `false` | `true` | disable Qt HiDPI scaling for RViz (fixes rendering glitches on some displays) |

## Hydraulic cylinders

Each cylinder is a `<name>_barrel` link on its base link and a `<name>_rod` link on its tip link,
attached by floating joints. `robot_state_publisher` and the joint GUIs ignore floating joints;
`excavator_hydraulics` publishes their TF so barrel and rod always point at each other's pin.

Pin positions and lengths are properties at the top of `excavator.urdf.xacro`. When changing them,
barrel and rod length must each be ≤ the shortest pin-to-pin distance over the joint range, and
together ≥ the longest. `test_urdf.py` checks this.

## Tests

```bash
colcon build
colcon test
colcon test-result --verbose
```

| Package | Tests |
|---|---|
| `excavator_description` | URDF is one tree with valid inertias, joints and limits match the table above, every cylinder fits over its joint's range, launch file builds |
| `excavator_gui` | URDF joint parsing, slider ↔ joint-position mapping and clamping (runs headless) |
| `excavator_hydraulics` | cylinder parsing, quaternion maths, barrel and rod point at each other's pin for a range of joint angles |

During development you can run one package's tests from its source folder with
`python3 -m pytest test`.
