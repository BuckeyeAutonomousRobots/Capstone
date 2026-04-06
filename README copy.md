# Pick Place Gazebo

This folder is a Gazebo-only ROS 2 workspace derived from `ros_backend0.6`.

It keeps the same UR5e + Robotiq Hand-E tabletop scene for pick-and-place:
- Gazebo world with the table and target cube
- UR5e + Hand-E robot description
- MoveIt Servo configuration for end-effector velocity control
- Reset manager that re-homes the robot and resets the cube pose

It intentionally removes the Unity, ROS-TCP, Quest, and VR hand-tracking pieces.

## Bringup

From this folder:

```bash
./pick_place_lifecycle.sh up_container_build
./pick_place_lifecycle.sh build_ws
./pick_place_lifecycle.sh bringup_all
```

./pick_place_lifecycle.sh build_ws
./pick_place_lifecycle.sh stop_nodes
./pick_place_lifecycle.sh start_sim
./pick_place_lifecycle.sh start_servo
./pick_place_lifecycle.sh start_control
./pick_place_lifecycle.sh run_keyboard


Open the Gazebo desktop at:

```text
http://localhost:6080/vnc.html
```

Then start the interactive keyboard teleop in your terminal:

```bash
./pick_place_lifecycle.sh run_keyboard
```

## Live camera image

The gripper model includes a Gazebo camera sensor on `/gripper_camera/image_raw`, but this Gazebo-only project does not start the camera bridges automatically. To view the live image, first make sure the desktop is open in your browser:

```text
http://localhost:6080/vnc.html
```

Then start the camera bridges from your host terminal:

```bash
docker exec -d pick_place_gazebo bash -lc '
source /opt/ros/humble/setup.bash
nohup ros2 run ros_gz_bridge parameter_bridge /gripper_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo >/tmp/gz_camera_info_bridge.log 2>&1 </dev/null &
nohup ros2 run ros_gz_image image_bridge /gripper_camera/image_raw >/tmp/gz_image_bridge.log 2>&1 </dev/null &
'
```

Then open the live viewer inside the container desktop:

```bash
docker exec -it pick_place_gazebo bash -lc '
source /opt/ros/humble/setup.bash
export DISPLAY=:1
ros2 run rqt_image_view rqt_image_view /gripper_camera/image_raw
'
```

Optional quick check:

```bash
docker exec pick_place_gazebo bash -lc '
source /opt/ros/humble/setup.bash
ros2 topic hz /gripper_camera/image_raw
'
```


## Windows and Linux note

This repo currently keeps the ARM-specific base image in `Dockerfile` for Apple Silicon Macs:

```dockerfile
FROM arm64v8/ros:humble-ros-base
```

If you are running on Linux or Windows with a typical x86_64 machine, change that line manually to:

```dockerfile
FROM ros:humble-ros-base
```

Windows users should run Docker in Linux container mode and use WSL2 or Git Bash for the `*.sh` scripts. This repo also includes `.gitattributes` to keep shell scripts and config files on LF line endings, which helps avoid Windows CRLF issues and will not interfere with the current Mac setup.

## Keyboard controls

```text
Move:
  w/s  +/- X
  a/d  +/- Y
  r/f  +/- Z

Rotate:
  i/k  +/- pitch
  j/l  +/- yaw
  u/o  +/- roll

Gripper:
  c    close
  v    open

Reset:
  b    run reset sequence and put cube back at start pose

Other:
  space  stop motion
  + / -  adjust linear speed
  ] / [  adjust angular speed
  h      print help
  q      quit teleop
```

## Workspace contents

- `simulation/`: Gazebo world and controller config
- `src/robotiq_hande_description`: Hand-E meshes and URDF
- `src/ur_hande_description`: UR5e + Hand-E combined robot description
- `src/ur_moveit_config`: MoveIt configuration used by Servo
- `src/servo_test_config`: Servo launch for the Gazebo simulation
- `src/teleop_bridge_msgs`: shared control message used by the teleop pipeline
- `src/pick_place_teleop`: keyboard teleop and control bridge nodes
