# Robotics_ROS_Project
2020/2021 Robotics course project, prof. Matteo Matteucci <br />
first part: computing an odometry giving the opportunity to choose between Euler and Runge-Kutta integration methods <br />
 <br />
![](odometry.gif)
second part: given a bag file, creating a map using gmapping(slam), fusing imu data and manufacturer's odometry with an ekf (extended-kalman-filter), then localizing the robot using a particle filter (amcl exploits lidar data to correct map->odom transform) <br />
<br />
![](gmapping.gif)
![](ekf_and_particle_filter.gif)
