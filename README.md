# Robotics_ROS_Project
2020/2021 Robotics course project, prof. Matteo Matteucci <br />
first part: computing an odometry giving the opportunity to choose between Euler and Runge-Kutta integration methods <br />
 <br />
![](odometry.gif)
second part: given a bag file, using gmapping(slam) to build a map, fusing imu data and manufacturer's odometry with an ekf (extended-kalman-filter), then localizing the robot inside the map using amcl (monte-carlo-localization exploits lidar data and filtered odometry to correct map->odom transform) <br />
<br />
![](gmapping.gif)
![](ekf_and_particle_filter.gif)
