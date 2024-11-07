# SLAM Gmapping 

This package contains a **ROS 2 Humble** wrapper for OpenSlam's Gmapping, based on [http://wiki.ros.org/gmapping](http://wiki.ros.org/gmapping).
The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called `slam_gmapping`. 
Using `slam_gmapping`, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.

Special thanks to Project MANAS for their contribution in migrating the SLAM method to ROS 2 Eloquent: https://github.com/Project-MANAS/slam_gmapping

## Launch

```bash
ros2 launch slam_gmapping slam_gmapping_launch.xml
```

The node `slam_gmapping` node takes in sensor_msgs/LaserScan messages and builds a map (nav_msgs/OccupancyGrid). 
The map can be retrieved via a ROS topic or service. 


## Subscribed Topics

- **`tf`** (`tf/tfMessage`):  
  Transforms necessary to relate frames for laser, base, and odometry.

- **`scan`** (`sensor_msgs/LaserScan`):  
  Laser scans to create the map from.

## Published Topics

- **`map_metadata`** (`nav_msgs/MapMetaData`):  
  Provides map metadata, which is latched and updated periodically.

- **`map`** (`nav_msgs/OccupancyGrid`):  
  Provides the map data, which is latched and updated periodically.

- **`~entropy`** (`std_msgs/Float64`):  
  Estimates the entropy of the map.

## Services

- **`dynamic_map`** (`nav_msgs/GetMap`):  
  Call this service to get the map data.

## Parameters

- **`~throttle_scans`** (`int`, default: `1`):  
  Process 1 out of every this many scans (set a higher number to skip more scans).

- **`~base_frame`** (`string`, default: `"base_link"`):  
  The frame attached to the mobile base.

- **`~map_frame`** (`string`, default: `"map"`):  
  The frame attached to the map.

- **`~odom_frame`** (`string`, default: `"odom"`):  
  The frame attached to the odometry system.

- **`~scan_topic`** (`string`, default: `"/scan"`):  
  The laser scan topic.

- **`~map_update_interval`** (`float`, default: `5.0`):  
  Interval in seconds between map updates. Lowering this number updates the occupancy grid more frequently, at the expense of computational load.

- **`~maxUrange`** (`float`, default: `80.0`):  
  Maximum usable range of the laser; beams are cropped to this value.

- **`~sigma`** (`float`, default: `0.05`):  
  Sigma used by the greedy endpoint matching.

- **`~kernelSize`** (`int`, default: `1`):  
  Kernel size used for searching correspondence.

- **`~lstep`** (`float`, default: `0.05`):  
  Optimization step in translation.

- **`~astep`** (`float`, default: `0.05`):  
  Optimization step in rotation.

- **`~iterations`** (`int`, default: `5`):  
  Number of iterations of the scan matcher.

- **`~lsigma`** (`float`, default: `0.075`):  
  Sigma of a beam used for likelihood computation.

- **`~ogain`** (`float`, default: `3.0`):  
  Gain used for likelihood evaluation to smooth resampling effects.

- **`~lskip`** (`int`, default: `0`):  
  Number of beams to skip in each scan. Only every `(n+1)`th laser ray is used for matching (0 = take all rays).

- **`~minimumScore`** (`float`, default: `0.0`):  
  Minimum score for considering the scan match outcome as good. Useful to prevent pose jumps in open spaces when using scanners with limited range (e.g., 5m).

- **`~srr`** (`float`, default: `0.1`):  
  Odometry error in translation as a function of translation (`rho/rho`).

- **`~srt`** (`float`, default: `0.2`):  
  Odometry error in translation as a function of rotation (`rho/theta`).

- **`~str`** (`float`, default: `0.1`):  
  Odometry error in rotation as a function of translation (`theta/rho`).

- **`~stt`** (`float`, default: `0.2`):  
  Odometry error in rotation as a function of rotation (`theta/theta`).

- **`~linearUpdate`** (`float`, default: `1.0`):  
  Process a scan each time the robot translates this distance.

- **`~angularUpdate`** (`float`, default: `0.5`):  
  Process a scan each time the robot rotates this angle.

- **`~temporalUpdate`** (`float`, default: `-1.0`):  
  Process a scan if the last processed scan is older than the specified update time (in seconds). A value less than zero disables time-based updates.

- **`~resampleThreshold`** (`float`, default: `0.5`):  
  Neff-based threshold for resampling.

- **`~particles`** (`int`, default: `30`):  
  Number of particles in the filter.

- **`~xmin`** (`float`, default: `-100.0`):  
  Initial map size in meters (minimum x-coordinate).

- **`~ymin`** (`float`, default: `-100.0`):  
  Initial map size in meters (minimum y-coordinate).

- **`~xmax`** (`float`, default: `100.0`):  
  Initial map size in meters (maximum x-coordinate).

- **`~ymax`** (`float`, default: `100.0`):  
  Initial map size in meters (maximum y-coordinate).

- **`~delta`** (`float`, default: `0.05`):  
  Map resolution in meters per occupancy grid cell.

- **`~llsamplerange`** (`float`, default: `0.01`):  
  Translational sampling range for likelihood computation.

- **`~llsamplestep`** (`float`, default: `0.01`):  
  Translational sampling step for likelihood computation.

- **`~lasamplerange`** (`float`, default: `0.005`):  
  Angular sampling range for likelihood computation.

- **`~lasamplestep`** (`float`, default: `0.005`):  
  Angular sampling step for likelihood computation.

- **`~transform_publish_period`** (`float`, default: `0.05`):  
  Interval (in seconds) between transform publications. Set to 0 to disable broadcasting transforms.

- **`~occ_thresh`** (`float`, default: `0.25`):  
  Threshold on occupancy values in gmapping. Cells with greater occupancy are marked as occupied (`100` in `sensor_msgs/LaserScan`).

- **`~maxRange`** (`float`):  
  Maximum range of the sensor. To represent regions with no obstacles within range as free space, set `maxUrange < maximum range of real sensor <= maxRange`.