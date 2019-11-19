# ambf_controller - WIP
RBE501 course project: Team 10


- All values in SI units (meters, radians, etc)

## TODO
- design logic for Iterative solution
- generate 4x4 homogeneous matrix for each link if required in most of the solvers
- solver for any two links in the chain.(currently only base<->tip works)

## Limitation
- Currently the solvers support only a chain (One base and one tip, but can have multiple branches within).
- Issue with config loader in AMBF : we have to use absolute path (With username) for meshes folder in example_robots file

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

**For FK**
```bash
rosrun ambf_controller ambf_controller.py --fk "1,2,3"
```

**For IK**
```bash
rosrun ambf_controller ambf_controller.py --ik "1,2,3,4,5,6"
```

### Develop environment
