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

For testing the dynamixel servos use:

~/catkin_ws/src/dynamixel_motor/dynamixel_driver/scripts$ python info_dump.py -p /dev/ttyUSB0 -b 57600 2
Pinging motors:

You should get: 

   Motor 1 is connected:
        Freespin: False
        Model ------------------- MX-28 (firmware version: 36)
        Min Angle --------------- 0
        Max Angle --------------- 4095
        Current Position -------- 3773
        Current Speed ----------- 0
        Current Temperature ----- 26°C
        Current Voltage --------- 6.4v
        Current Load ------------ 0
        Moving ------------------ False
...

If checksum errors or wrong packet prefix errors arise, this is usually caused by wrong voltage or cabling, e.g:

[INFO] [WallTime: 1449485292.823794] pan_tilt_port: Pinging motor IDs 1 through 5...
[ERROR] [WallTime: 1449485293.135957] Exception thrown while getting attributes for motor 1 - Checksum received from motor 1 does not match the expected one (215 != 247)
[ERROR] [WallTime: 1449485293.197830] Exception thrown while getting attributes for motor 2 - Checksum received from motor 2 does not match the expected one (220 != 221)
[ERROR] [WallTime: 1449485293.211449] Exception thrown while getting attributes for motor 2 - Invalid response received from motor 2. Wrong packet prefix ['\xfd', '\xff']
[ERROR] [WallTime: 1449485293.215802] Exception thrown while getting attributes for motor 2 - Invalid response received from motor 2. Wrong packet prefix ['\x7f', '\xfe']

