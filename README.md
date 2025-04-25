<a id="readme-top"></a>
<!-- PROJECT LOGO -->
<br />
<div>

# DLO: 3D Direct LIDAR Odometry based on Fast Truncated Distance Field

A Direct LIDAR Odometry (DLO) able to deal with raw 3D LIDAR data online, avoiding the need of LIDAR feature selection and tracking, simplifying the odometry pipeline and easily generalising to all types of scenarios. The method is based on the use of Fast Truncated Distance Field (Fast-TDF) as a convenient tool to represent the environment, enabling solving the LIDAR point-cloud registration as a nonlinear optimisation process without the need for selecting/tracking LIDAR features in the input data and updating the environment map at constant time independently of the size of the map itself.
<p align="center">
  <img src="media/map_college.png" width="45%" />
  <img src="media/map_eee02.png" width="45%" />
</p>

## 1. Prerequisites


The program runs on **ROS 2 Humble** and **Ubuntu 24.04** or higher, in addition to the following dependencies required to build and run the project:

- `ament_cmake`
- `rclcpp`
- `message_filters`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `pcl_ros`
- `pcl_conversions`
- `PCL` (Common, Filters components)
- `Boost` (Thread, Chrono components)
- `glog`
- `Ceres`
- `OpenMP`
- `Eigen3`
- `std_srvs`
- `ANN_LIB` (Approximate Nearest Neighbor library)

For convenience, you can install all dependencies by running the following command **inside the package folder**:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 2. Installation


To install and build the project, simply clone the repository as follows:

   ```bash
   git clone https://github.com/robotics-upo/dlo3d.git
   cd ..
   colcon build
   source install/setup.bash
```

## 3. Running the Code
The code can be launched automatically using one of the available launch files. There is a generic launch file, **dlo3d_launch.py**, which serves as a template for adapting the configuration to the specific dataset. Additionally, there are two predefined launch files tailored for the VIRAL and College datasets.

To launch the code, use the following example command:
  ```bash
ros2 launch dlo3d dlo3d_launch.py
```
This command will start the node and prepare it to receive information via the topics. The node will remain in a waiting state until data is published. If you wish to additionally launch a pre-recorded bag file, you can specify the bag_path parameter as shown below:


  ```bash
ros2 launch dlo3d dlo3d_launch.py bag_path:='bag_path1/bag.db3
```
Along with the node and bag file, RViz visualization will also be launched to display a 3D representation of the environmen

**3.1 Node Configuration Parameters**

The dlo3d_node requires a series of configuration parameters to operate correctly, which are related to the dataset and the vehicle being used. These parameters are as follows:
- **in_cloud_aux**
- **in_cloud**
- **hz_cloud**
- **in_imu**
- **hz_imu**
- **calibration_time**
- **aux_lidar_en**
- **gyr_dev**
- **gyr_rw_dev**
- **acc_dev**
- **acc_rw_dev**
- **base_frame_id**
- **odom_frame_id**
- **map_frame_id**
- **keyframe_dist**
- **keyframe_rot**
- **tdfGridSizeX_low**
- **tdfGridSizeX_high**
- **tdfGridSizeY_low**
- **tdfGridSizeY_high**
- **tdfGridSizeZ_low**
- **tdfGridSizeZ_high**
- **solver_max_iter**
- **solver_max_threads**
- **min_range**
- **max_range**
- **pc_downsampling**
- **robust_kernel_scale**

These parameters allow you to fine-tune the node’s behavior, including settings related to the input cloud topics, IMU data, grid size, calibration, and solver configurations, among others.