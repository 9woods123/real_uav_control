Real UAV control， using the vicon to locate the uav.  The position with yaw can be set to control the uav, the target point is send to mavros , which processes and delivers the msgs to the PX4.

Thanks to Jianjun Gui and Haoxin Zhang, who  helped a lot for this work.  The work is fineshed in Defense Innovation Institute, Chinese Academy of Military Science, 2021.




Welcome to the real_uav_control wiki!

For using this,  some dependences are followed

`sudo apt-get  install  ros-melodic-vrpn`

`sudo apt-get install  ros-melodic-mavros-msgs `

make the workshop 

`catkin_make`

and launch the uav control nodes:

`source  your_catkin_name/devel/setup.bash `

`roslaunch  move_control  real_world_uav.launch`

