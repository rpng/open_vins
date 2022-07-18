
These trajectories are from the RTK GPS.
Ideally, would need to be transformed to the IMU frame.
Additionally, there is no timeoffset taken into account which will cause errors too.


**Don't use these to evaluate error, just use theses to visualize!! If you do compare, you should only use the ATE and say that you are comparing agaist a non-perfect groundtruth. Additionally you should filter the poses based on the positionally accuracy (see the last columns in each text file).**


If you are looking at these groundtruth files, you might be interested in:
- https://github.com/rpng/gps_path_pub

