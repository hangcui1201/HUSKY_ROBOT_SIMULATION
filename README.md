## Exercise 1

#### $ roslaunch husky_gazebo husky_empty_world.launch world_name:=worlds/willowgarage.world
#### $ roslaunch husky_gazebo husky_empty_world.launch world_name:=worlds/robocup14_spl_field.world
#### $ rostopic pub -r 10 /cmd_vel geometry_msgs/Twist -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

#### $ roslaunch husky_drive husky_drive.launch
#### $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py


## Exercise 2

#### $ roslaunch husky_highlevel_controller husky_drive.launch
#### $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py


## Exercise 3


