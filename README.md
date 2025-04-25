<a id="readme-top"></a>
<!-- PROJECT LOGO -->
<br />
<div>

# DLO: 3D Direct LIDAR Odometry based on Fast Truncated Distance Field
 A Direct LIDAR Odometry (DLO) able to deal with raw 3D LIDAR data online, avoiding the need of LIDAR feature selection and tracking, simplifying the odometry pipeline and easily generalising to all type of scenarios. The method is based on the use of Fast Truncated Distance Field (Fast-TDF) as convenient tool to represent the environment than enables solving the LIDAR point-cloud registration as a nonlinear optimisation process without the need of selecting/tracking LIDAR features in the input data and updating the environment map at constant time independently of the size of the map itself.
 