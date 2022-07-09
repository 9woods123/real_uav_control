Real UAV control， using the vicon to locate the uav.  The position with yaw can be set to control the uav, the target point is send to mavros , which processes and delivers the msgs to the PX4.

Thanks to Jianjun Gui and Haoxin Zhang, who  helped a lot for this work.  The work is fineshed in Defense Innovation Institute, Chinese Academy of Military Science, 2021.




Welcome to the real_uav_control wiki!

For using this,  some dependences are followed

`sudo apt-get  install  ros-melodic-vrpn`

`sudo apt-get install  ros-melodic-mavros-msgs `

make the workshop 


```
cd your_catkin_name

git clone https://github.com/9woods123/real_uav_control.git

catkin_make

```

and launch the uav control nodes:

`source  your_catkin_name/devel/setup.bash `

`roslaunch  move_control  real_world_uav.launch`


The uav_simple_control.cpp is a tutorial code.  Fix your algorithm , sending the target point like it does and replace the `uav_simple_control`
