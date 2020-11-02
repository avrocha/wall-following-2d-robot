# wall-following-2d-robot
ROBO@FEUP - 2nd Assignment - Wall Following 2D Robot using ROS and STDR simulator

# STDR Simulator Configuration
Copy all the contents of 'stdr_resources/maps' to the respective folder inside the stdr_resources package (use 'roscd stdr_resources/maps')
Copy all the contents of 'stdr_resources/resources/robots' to the respective folder in the stdr_resources package (use 'roscd stdr_resources/resources/robots')

# Build
1)
```shell
$ cd ~/catkin_ws/src/wall_following_2d_robot/src
$ chmod +x follow_wall.py
```
2)
```shell
$ cd ~/catkin_ws
$ catkin_make
```

# Run
```shell
'roslaunch stdr_launchers wall_follow_robot.launch'
```

```shell
'rosrun wall_following_2d_robot follow_wall.py'
```