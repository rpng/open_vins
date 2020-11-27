## Credit / Licensing

This forked repository aimes to conduct research on camera-IMU calibration. Most changes compared with the original was done in the current directory, `ov_calib` as well as some source files were added to `../ov_msckf` and `../ov_core`. For any inquiries in this changes, please contact Jongwon Lee ([jongwon5@illinois.edu](jongwon5@illinois.edu)).


The original project, [OpenVINS](https://github.com/rpng/open_vins), was developed by the Robot Perception and Navigation Group (RPNG) at the University of Delaware. Please cite the following:

```
@Conference{Geneva2020ICRA,
  Title      = {OpenVINS: A Research Platform for Visual-Inertial Estimation},
  Author     = {Patrick Geneva and Kevin Eckenhoff and Woosik Lee and Yulin Yang and Guoquan Huang},
  Booktitle  = {Proc. of the IEEE International Conference on Robotics and Automation},
  Year       = {2020},
  Address    = {Paris, France},
  Url        = {\url{https://github.com/rpng/open_vins}}
}
```

The codebase is licensed under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).

## How to execute

```
$ cd {OPENVINS_HOME_DIRECTORY}
$ catkin build
$ source devel/setup.bash
$ roslaunch ov_calib run_sim.launch
```

To execute a proper simulation, please modify `launch/run_sim.launch` on your own.