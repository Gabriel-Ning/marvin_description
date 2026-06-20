# marvin_description

ROS 2 description package for the Marvin CCS M6 bimanual arm embodiment.

This package contains the canonical arm-only URDF/xacro model, meshes, and
description-only launch files. It is intentionally hardware-free: no SDK
transport, no controller manager launch, no gripper hardware, and no real motion
commands live here.

## Scope

Current model:

- Marvin CCS M6 bimanual arm, controller type `1017`
- 14 revolute arm joints: `Joint1_L..Joint7_L` and `Joint1_R..Joint7_R`
- Native arm flange frames: `flange_L` and `flange_R`
- Native arm terminal frames are `flange_L` and `flange_R`; no tool frames are
  defined while the real robot has no attached gripper or tool.
- Stand/base frames: `base_link`, `column_link`, `tracking_base_link`

Deferred modules:

- grippers and gripper controllers
- MoveIt configuration
- custom controllers from the higher-level control stack

## Package Layout

```text
marvin_description/
  launch/
    visualize_marvin.launch.py
  meshes/
    base/
    m6/
  urdf/
    marvin.urdf.xacro
    marvin_with_gripper.urdf.xacro
    parts/
      marvin_stand.xacro
      marvin_left_arm.xacro
      marvin_right_arm.xacro
      marvin_bimanual_arm.xacro
      marvin_arm.ros2_control.xacro
    rviz2.rviz
```

## Xacro Entry Point

Use `urdf/marvin.urdf.xacro` as the canonical entry point.

Supported arguments:

```text
connected_to:=world
xyz:=0 0 0
rpy:=0 0 0
ros2_control:=true
use_fake_hardware:=true
hardware_plugin:=marvin_hardware_interface/MarvinBimanualArmHardware
```

The expected tree is:

```text
world
  -> base_link
      -> column_link
      -> tracking_base_link
      -> Base_L -> ... -> Link7_L -> flange_L
      -> Base_R -> ... -> Link7_R -> flange_R
```

`flange_L` and `flange_R` are native arm frames. Do not add extra flange links
above them in this package.

## ros2_control Block

`urdf/parts/marvin_arm.ros2_control.xacro` exposes only the 14 arm joints.

Command interfaces:

```text
position
velocity
effort
```

The current validated controller path is position-only JointTrajectoryController
usage. `velocity` and `effort` command interfaces are reserved for future
controllers. The hardware mode-switch contract rejects claims for them until
their corresponding write modes are implemented.

State interfaces:

```text
position
velocity
effort
```

Hardware selection follows the fake/real switch pattern:

```text
use_fake_hardware:=true   -> mock_components/GenericSystem
use_fake_hardware:=false  -> hardware_plugin
```

The default is fake hardware. This package must not connect to the vendor SDK.

## Build

From a ROS 2 workspace containing this package:

```bash
colcon build --symlink-install --packages-select marvin_description --cmake-args -Wno-dev
source install/setup.bash
```

## Validate The URDF

```bash
xacro $(ros2 pkg prefix marvin_description)/share/marvin_description/urdf/marvin.urdf.xacro \
  ros2_control:=true \
  use_fake_hardware:=true \
  > /tmp/marvin.urdf

check_urdf /tmp/marvin.urdf
```

Expected:

- xacro expands successfully
- `check_urdf` succeeds
- one connected tree from `world`
- `flange_L` and `flange_R` exist
- 14 revolute arm joints
- no gripper joints

## Visualize

```bash
ros2 launch marvin_description visualize_marvin.launch.py
```

This launch starts `robot_state_publisher`, `joint_state_publisher` or
`joint_state_publisher_gui`, and RViz2. It does not start ros2_control or any
hardware driver.

## Real Hardware Notes

The real controller currently uses the Marvin CCS M6 4.0 SDK configuration.
Future SDK kinematics, IK, calibration, and real read/write gates should use
`ccs_m6_40.MvKDCfg`, not older 3.1/reference configuration files.

The native flange transform is aligned with `ccs_m6_40.MvKDCfg`, including its
terminal MDH segment `90, 0, 95, 90`. Offline FK comparison at zero and two
non-zero joint configurations agrees within `3e-6 m` and `2.3e-5 rad`.

`m_FB_Joint_SToq` is documented by the vendor only as feedback joint torque. Its
unit and scaling are not specified. The exposed effort state is therefore
provisional raw feedback and must not be used for force/torque control until a
known-load calibration confirms SI `N*m` semantics.
