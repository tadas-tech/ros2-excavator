# ros2-excavator

ROS 2 (Lyrical) model of a tracked excavator, viewable in RViz and driven with joint sliders.

The model is built from primitive shapes in xacro — no mesh files. The hydraulic cylinders
extend and retract with the joints.

## Packages

| Package | Contents |
|---|---|
| `excavator_description` | xacro model, launch file, RViz config |
| `excavator_gui` | Qt slider window; one slider per joint, read from the URDF |
| `excavator_hydraulics` | Node that publishes TF for the cylinder barrels and rods, and their strokes |
| `excavator_msgs` | `CylinderState` / `CylinderStates` messages |

## Joints

| Joint | Moves | Range (rad) |
|---|---|---|
| `swing_joint` | upper body on the tracks | continuous (no limit; slider covers −π … π) |
| `boom_joint` | boom | −0.6 … 1.1 |
| `arm_joint` | arm (stick) | −2.6 … 0.3 |
| `bucket_joint` | bucket | −2.2 … 1.0 |

Positive angles raise the boom, arm and bucket. For the bucket that means opening it (tipping the
load out); negative curls it in towards the machine.

## Slider GUI

One slider per joint, plus buttons that move the joints smoothly to a preset pose. All joints
arrive together without exceeding their URDF velocity limits; dragging a slider stops the motion.

| Button | Pose |
|---|---|
| Transport | arm folded in, bucket curled and carried ~0.5 m above the ground |
| Dig | arm reaching out ~9 m, bucket opened, teeth just into the ground |
| Dump | swung 90° to the cab side, bucket ~3.8 m up and tipped out |
| Reset | all joints back to 0 |

The **Bucket tip** panel moves the tip of the middle tooth (the `bucket_tip` frame) instead of
single joints. Drag a slider or type a value and press Enter:

| Field | Meaning |
|---|---|
| reach | distance from the swing axis, along the swing direction (m) |
| height | height above the ground (m) |
| bucket angle | pitch of the bucket (boom + arm + bucket angle); 0° as in the Reset pose, + tips it open |

Changing one field keeps the other two fixed — e.g. lowering the height moves the tip straight
down without tilting the bucket. Boom, arm and bucket angles are solved in closed form
(`src/excavator_gui/excavator_gui/kinematics.py`); if no joint angles within the limits put the tip
there, the panel says "Out of reach" and nothing moves. Swing stays on its own slider.

A **Cylinders** panel under the buttons shows each hydraulic cylinder's stroke (0 % fully
retracted, 100 % fully extended), its pin-to-pin length and its speed.

The poses are in `src/excavator_gui/excavator_gui/poses.py`; `test_urdf.py` checks they are within
the joint limits and that the bucket ends up at the right height.

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

`excavator_hydraulics` also publishes `/cylinder_states` (`excavator_msgs/CylinderStates`, 30 Hz):
for each cylinder its pin-to-pin `length`, the `min_length` / `max_length` over its joint's range
(fully retracted / extended), `stroke` = length − min_length, and `velocity` (m/s, + extending).

```bash
ros2 topic echo /cylinder_states
```

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
| `excavator_description` | URDF is one tree with valid inertias, joints and limits match the table above, every cylinder fits over its joint's range, bucket faces the backhoe way, `bucket_tip` is on the teeth and the GUI's tip kinematics match the URDF, presets are reachable and put the bucket at the right height, launch file builds |
| `excavator_gui` | URDF joint parsing, slider ↔ joint-position mapping and clamping, preset poses and smooth motion, bucket-tip forward/inverse kinematics and panel, cylinder panel (runs headless) |
| `excavator_hydraulics` | cylinder parsing, quaternion maths, barrel and rod point at each other's pin for a range of joint angles, cylinder length range, stroke and speed |

During development you can run one package's tests from its source folder with
`python3 -m pytest test`.
