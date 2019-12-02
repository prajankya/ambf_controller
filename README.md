# ambf_controller

- All values in SI units (meters, radians, etc)

## Limitation
- Currently the solvers support only a chain (One base and one tip, but can have multiple branches within).

## Changes to existing AMBF - YAML format
- Added `solver` parameter to the root of the robot(please check example yaml in this repo)

## Install
This project depends on AMBF to be installed first, so follow [https://github.com/WPI-AIM/ambf/](https://github.com/WPI-AIM/ambf/) to install.

**Note**: Add the `setup.bash` to source as suggested at the end of build section in link given above.

Now run below to install this repository

```bash
cd catkin_ws/src
git clone https://github.com/prajankya/ambf_controller.git
rosdep install --from-paths ./ambf_controller/libs/** --ignore-src
rosdep install --from-paths ./ambf_controller --ignore-src
catkin_make
```

### Running

Controller is a rosnode, it can be started using the following command.

```bash
rosrun ambf_controller ambf_controller ~/ambf/ambf_models/descriptions/multi-bodies/robots/blender-kuka.yaml
```

**Note** : The file name passed end of the command shown above should be the same robot which is currently loaded in AMBF.

**To Move robot to a pose**
```bash
rostopic pub /ambf/setPose geometry_msgs/Pose "position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0"
```

**To Move robot with given Joint Parameters**
```bash
rostopic pub /ambf/setJointParams std_msgs/Float64MultiArray "layout:
  dim:  
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:   
- 0.0
- 0.0
- 0.0
- 0.0   
- 0.0
- 0.0
- 0.0
```