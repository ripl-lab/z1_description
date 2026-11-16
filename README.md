# z1_description

The Unitree Z1 arm for ROS 2: xacro, meshes, a MuJoCo model, and the ends of arm the lab bolts to it.

The arm and Unitree's gripper are copies from the [official `unitree_ros`](https://github.com/unitreerobotics/unitree_ros/tree/8ebecd883a842a4631fcfa98933fae586aeb6709/robots/z1_description).
They are changed for ROS 2 control and Gazebo Sim. The lab parts come from the lab CAD.

## Variants

One xacro argument, `variant`, picks what sits on the gripper motor (the 7th motor).

| `variant` | On the gripper motor | Cameras | `z1_tcp` |
| --- | --- | --- | --- |
| `unitree_gripper` (default) | Unitree's two jaws | D435i at the hand-eye mount | 140 mm past the J6 flange |
| `bota_probe` | U-bracket, Bota Medusa F/T sensor, Ø15 ball probe | D405 below the face plate, D435i on a mast | ball centre, 201.5 mm past the flange |
| `robotiq_2f85` | U-bracket, Robotiq 2F-85 on its coupling | D405 on a portal over the gripper | between the open pads, 252 mm past the flange |

`bota_probe` and `robotiq_2f85` sit on the bare motor, with Unitree's clamshell removed. The U-bracket bolts to
the two end faces of the motor.

Two more arguments work with every variant:

- `with_gripper` (default `true`): the gripper motor is wired, so `jointGripper` is in the URDF and in
  ros2_control. With `unitree_gripper` and `false`, the arm ends at `link06`. With the other variants and `false`,
  the motor stays as a fixed body.
- `with_camera` (default `true`): the variant's cameras and their printed mounts. `false` leaves them out, also of
  the gravity model of `z1_hardware_interface`.

`z1_bringup` and `z1_moveit` take the same `variant` argument.

## View a variant

In RViz, with a slider for each joint. From b2_ws:

```bash
pixi run ros2 launch z1_description display.launch.py variant:=bota_probe   # or unitree_gripper, robotiq_2f85
pixi run ros2 launch b2_description display.launch.py model:=b2z1_bota_probe  # the arm on the B2
```

- Every frame shows its axes and its name: x red, y green, z blue. The model is 60 % opaque, so the frames inside
  a part stay visible. Turn single frames on or off under TF, Frames.
- Mouse: left drag orbits, middle drag (or Shift and left drag) pans, the wheel zooms. To centre the view on a
  part, press F, then click the part.
- If RViz opens no window under Wayland, start it with `QT_QPA_PLATFORM=xcb` before the command.

As pictures, with no ROS nodes: four views of each variant's end of arm, every frame drawn and named, in the
`gripperStator` frame.

```bash
pixi run python src/z1_description/scripts/render_frames.py               # all variants, PNG files in ./z1_frames
pixi run python src/z1_description/scripts/render_frames.py bota_probe --out /tmp/frames
```

The live wrench of the Bota sensor has its own viewer in b2_ws: `pixi run viz-bota` (`tools/bota_ft/viz_ft.py`).
It draws the force and the torque in the same frame as `bota_wrench`.

### Frames

Every variant has `link00` to `link06`, `gripperStator` and `z1_tcp`. `gripperStator` is fixed to `link06` at the
J6 flange face: x out along J6, y along the motor axis, z up at the zero pose. The lab CAD calls this frame G.

| Frame | Variant | What it is |
| --- | --- | --- |
| `bota_wrench` | `bota_probe` | The frame of the Bota wrench: on the axis at the tool face, z out of the tool side, y towards the M8 connector. The sensor is turned 90° about J6, so y points to the right (-y of `gripperStator`). |
| `probe_ball`, `probe_tip` | `bota_probe` | The ball centre, and the front of the ball. |
| `camera_link` | `unitree_gripper`, `bota_probe` | The D435i. |
| `d405_link` | `bota_probe`, `robotiq_2f85` | The D405. |

The camera URDF ends at `<name>_link`. The RealSense driver publishes the optical frames from the camera's own
extrinsics. Run the driver with `camera_name:=camera` for the D435i and `camera_name:=d405` for the D405.

### Masses

The lab parts add these masses on the gripper motor:

| Part | Mass | Source |
| --- | --- | --- |
| Bota U-bracket | 86.6 g | CAD volume, print model |
| Bota Medusa, serial | 113 g | Bota datasheet |
| Ball probe | 17.5 g | CAD volume, print model |
| D405 saddle | 16.0 g | CAD volume, print model |
| D435i mast | 45.4 g | CAD volume, print model |
| Robotiq U-bracket | 94.9 g | CAD volume, print model |
| 2F-85 with coupling | 921 g | Robotiq UR manual, with its centre of mass |
| D405 portal on the Robotiq bracket | 41.4 g | CAD volume, print model |

The print model is PETG with 1.8 mm walls and 40 % gyroid infill. realsense2_description gives each camera 72 g.

These numbers are not weighed:

- The bare gripper motor keeps Unitree's mass for the motor with its fixed jaw, 0.526 kg. The bare motor is
  lighter.
- In the lab variants, `gripperMover` is the output disk. It gets 10 g, because Gazebo Sim drops a link with no
  mass.
- The printed parts follow the print model, not a scale.

Weigh the parts before you trust the gravity model with a lab variant.

## Use the Z1 in another robot

`urdf/z1_macro.xacro` holds the macro `z1`. It fixes `link00` to a parent link. `b2_description` uses it to put the
arm on the B2:

```xml
<xacro:include filename="$(find z1_description)/urdf/z1_macro.xacro"/>
<xacro:z1 parent="base_link" variant="$(arg variant)"
          with_gripper="$(arg with_gripper)" with_camera="$(arg with_camera)">
    <origin xyz="0.34218 0 0.2425" rpy="0 0 0"/>
</xacro:z1>
```

The fixed joint is `z1_mount`. Set `mount_joint` to give it another name.

## Files

| Path | What it is |
| --- | --- |
| `urdf/z1.urdf.xacro` | The arm alone on `world`, with ros2_control and the Gazebo Sim plugin. Arguments: `variant`, `with_gripper`, `with_camera`, `sim_ignition`, `name`, `prefix`, `controllers`. |
| `urdf/z1_macro.xacro` | The macro `z1`: the arm and one variant. |
| `urdf/z1_arm.xacro` | `link00` to `link06`. |
| `urdf/z1_gripper.xacro` | Unitree's gripper and the wrist D435i. |
| `urdf/lab/` | The lab variants and the pieces they share. |
| `urdf/const.xacro` | Unitree's masses, inertias and joint limits. |
| `urdf/z1.ros2_control.xacro` | The ros2_control block. |
| `config/camera_mount.yaml` | The D435i mount of `unitree_gripper`, from hand-eye calibration. |
| `config/lab/*.yaml` | Masses, frames and camera mounts of each lab variant. Generated. |
| `meshes/visual/`, `meshes/collision/` | Unitree's meshes, in m. |
| `meshes/lab/` | The lab parts in the `gripperStator` frame, in mm. `meshes/lab/collision/` holds their convex hulls. Generated. |
| `mjcf/` | The MuJoCo model of the Z1 with Unitree's gripper and the D435i. The lab variants have no MuJoCo model yet. |
| `launch/display.launch.py`, `config/display.rviz` | RViz with joint sliders. |
| `scripts/render_frames.py` | Pictures of each variant with every frame drawn. |

## Regenerate

**Lab variants.** The lab CAD lives in b2_ws, in `cad/gripper-prototype`. Its script writes `config/lab/` and
`meshes/lab/` from the build123d parts and from the camera poses of the wrist-camera page:

```bash
cd ~/b2_ws/cad/gripper-prototype
pixi run python export_z1_description.py    # about 30 s
```

**D435i mount.** A hand-eye run writes `config/camera_mount.yaml` and `mjcf/d435i.xml`:

```bash
cd ~/b2_ws
pixi run z1-camera-mount datasets/z1/handeye/<run>
```

## Known gaps

- The 2F-85 is one rigid body, drawn open. Its fingers have no joints, and the Robotiq driver publishes nothing
  into this URDF.
- The collision mesh of each lab part is its convex hull. The 2F-85 has three hulls: the base and one per finger,
  so the space between the pads stays free.
- MoveIt's SRDF in `z1_moveit` turns off the collision checks between parts fixed to `gripperStator`. Parts and
  cameras are still checked against `link00` to `link05`.
