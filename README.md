# robotnik_ptu
Robotnik Pan-Tilt Unit (standard dynamixel servos)

The package is intended for using 2 pan-tilt units each one with one ASUS Xtion RGBD device. It allows using two JointTrajectoryAction servers:
  -Robotnik rt_traj_exe (from robotnik_trajectory_suite) (launch/rtc launches)
  -Dynamixel joint trajectory controller (launch/dynamixel launches)

Configure the xtion_rgbd.launch file with the device ids of your xtion cameras. The easiest way to select a device is by its bus.  

Launch with 

>roslaunch robotnik_ptu robotnik_ptu.launch

This starts the 2 xtion and 2 ptu units by default

In order to see the PointClouds in rviz transforms to the optical frames should be present, so launch something that publishes them, e.g.

>roslaunch summit_xl_description summit_xl_hls_state_robot.launch

See the rgb images and pointclouds in rviz

>rosrun rviz rviz 

The ptu units can be moved from MoveIt!, e.g. with 

>roslaunch summit_xl_moveit demo_rtc.launch (remember to change the config/controllers.yaml file if you want to change to the dynamixel joint trajectory controller)


