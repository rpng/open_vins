

# Open VINS


Welcome to the Open VINS project!
The Open VINS project houses some core computer vision code along with a state-of-the art filter-based visual-inertial estimator.
The core filter is an [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) which fuses inertial information with sparse visual feature tracks.
These visual feature tracks are fused leveraging the [Multi-State Constraint Kalman Filter (MSCKF)](https://ieeexplore.ieee.org/document/4209642) sliding window formulation which allows for 3D features to update the state estimate without directly estimating the feature states in the filter.
Inspired by graph-based optimization systems, the included filter has modularity allowing for convenient covariance management with a proper type-based state system.
Please take a look at the feature list below for full details on what the system supports.


* Github project page - https://github.com/rpng/open_vins
* Documentation - TBD
* Publication reference - TBD




## Project Features


* Sliding window visual-inertial MSCKF
* Modular covariance type system
* Comprehensive documentation and derivations
* Five different feature representations
    1. Global XYZ
    2. Global inverse depth
    3. Anchored XYZ
    4. Anchored inverse depth
    5. Anchored MSCKF inverse depth
* Calibration of sensor intrinsics and extrinsics
    * Camera to IMU transform 
    * Camera to IMU time offset
    * Camera intrinsics
* Environmental SLAM feature
    * OpenCV ARUCO tag SLAM features
    * Sparse feature SLAM features
* Visual tracking support
    * Monocular camera
    * Stereo camera
    * KLT or descriptor based
* Static IMU initialization (sfm will be open sourced later)
* Out of the box evaluation on EurocMav and TUM-VI datasets




## Credit / Licensing

This code was written by the [Robot Perception and Navigation Group (RPNG)](https://sites.udel.edu/robot/) at the University of Delaware.
If you have any issues with the code please open an issue on our github page with relevant implementation details and references.
For researchers that have leveraged or compared to this work, please cite the following:
```txt
@article{TBD,
  author    = {},
  title     = {},
  journal   = {},
  volume    = {},
  year      = {2019},
}
```


The codebase is licensed under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).


