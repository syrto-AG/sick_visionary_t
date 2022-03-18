Travis-CI: [![Build Status](https://travis-ci.org/SICKAG/sick_visionary_t.svg?branch=indigo-devel)](https://travis-ci.org/SICKAG/sick_visionary_t)

# sick_visionary_t
Repository for ROS-enabled 3D sensors from Sick.

# Start 

Enter the sensor ip address in sick_visionary_t/sick_visionary_t_driver/launch/sick_visionary_t_driver.launch file.

The point cloud is sent on the topic /sick_visionary_t_driver/points.


## Note

You may need to install the cv bridge package.

```bash
 sudo apt-get install ros-noetic-cv-bridge
```