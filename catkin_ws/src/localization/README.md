# Localization
In this section, I implement [Localization](https://pythonrobotics.readthedocs.io/en/latest/modules/localization.html) includes EKF, UKF, particle filter and histogram filter
[![Localization](https://img.youtube.com/vi/hp-lATCbfkw/0.jpg)](https://www.youtube.com/watch?v=hp-lATCbfkw)
# How to use
1. Run roscore
```
roscore
```
2. Open simulation world
```
webots webots_ros_PythonRobotics/sim_world/worlds/turtlebot3.wbt
```
2. Check your PID of ros controller in webots (PID will be used in next step.)
```
rosservice list
```
You will see many rosservices like this:
```
/TurtleBot3Burger_37899_username_computer_name/set_logger_level
```
```TurtleBot3Burger_37899_username_computer_name``` is your PID of ros controller

3. Running nodes with your PID
```
roslaunch localization ekf.launch pid:=TurtleBot3Burger_37899_username_computer_name
```
4. Teleporate your Turtlebot3 to see the result
# Visualization
You can see the results in rivz. There are three topics being published:
1. ```/xDR``` : Dead-reckoning state

2. ```/xEst```: Estimated state from filter

3. ```/xTrue```: True state


