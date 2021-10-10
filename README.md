# CRANE_X7_DRAKE

This is an example manipulation package of for a robot manipulator based on [Drake](https://github.com/RobotLocomotion/drake) with ROS2.


Build and run an example with Docker:
```
docker build -t crane_x7_drake .
docker run -p 6080:80 --shm-size=512m crane_x7_drake
```
You can see the gui from a web browser (VNC)
```
http://127.0.0.1:6080/
```
and run the gazebo simulation on the VNC
```
. /opt/ros/foxy/setup.sh
. /home/ros2_ws/install/setup.sh
ros2 launch crane_x7_gazebo crane_x7_gazebo.launch.py gui:=true
```