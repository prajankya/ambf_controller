# ambf_controller
RBE501 course project: Team 10


## Limitation
- Currently the solvers support only a chain (One base and one tip, but can have multiple branches within).

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


### Develop environment
