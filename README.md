# Autonomous-and-Mobile-Robotics-project
in this project we exploited the turtlebot3 burger robot understanding how to make it able to autonomously plan, navigate 
and localize itself by using the Robotics Operating System (ROS).
## First Task
At the beginning, we set Gazebo and RViz to simulate the physical behavior of the robot and to start the visualization of 
what it sees through its sensors.
In order to run it, do the following command in the project folder
```
catkin_make
```
source the package installation:
```
source devel/setup.bash
```
and then start the task as
```
roslaunch first_task first_task.launch
```
## Second Task
After we have set Gazebo and RViz, we run the SLAM algorithm and the explore node in order to
make the robot moving around the environment to build the map.  
To launch the second task use the following command
```
roslaunch second_task second_task.launch
```
then, the robot starts moving around building the map of the house.
## Third Task
Supposing that the map is known, now the robot must localize itself without any prior knowledge on
where it is exactly. In order to do that, we run the *generalized montecarlo localization* algorithm and the global localization
services to spread particles all over the map.  
To run the nodes use the following command
```
roslaunch third_task third_task.launch
```
Now, we have to run the *send_goal* node. In order to do that, we have to open the python executable in 
the specific package.  
By using the command
```
roscd send_goal/src
```
now we are inside the package.  
Then, to run the executable just digit
```
python3 send_goal.py
```
## Fourth Task
When the robot has finally localized itself in the map, we launch the last nodes to sanitize the rooms.  
The environment is the same of the previous task, but with the following commands 
```
roscd fourth_task/src
python3 send_goal_sanitization.py
```
and 
```
python3 sanitization.py
```
the robot start moving to sanitize the selected rooms by visualizing the results in an OccupancyGrid.