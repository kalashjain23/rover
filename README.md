# **Rover**

## **Idea**
A rover capable of performing **mapping, localizing, and navigating** itself through the map!
 
*This package has been created and tested on Ubuntu 22.04 with ROS2 Humble, Gazebo 11.10.2, Rviz2, and Nav2 stack.*

## **How to build**
*Creating a workspace to build the package*
```
mkdir -p ~/rover_ws/src && cd ~/rover_ws/src
```
*Cloning the package*
```
git clone https://github.com/kalashjain23/rover
cd ~/rover_ws
```
*Installing the dependencies and building the workspace*
```
rosdep install --from-paths src -y --ignore-src
colcon build
```

## **Mapping**
First, edit the [mapper_params_online_async.yaml](https://github.com/kalashjain23/rover/blob/d850479213e026ad81164b0ba3b702957188e3ad/config/mapper_params_online_async.yaml#LL17C23-L17C23) file to set the mode to mapping.
```
    mode: mapping   # Changed from visualization

    map_file_name: test_name    # comment this

    map_start_at_dock: true     # comment this
```
Now you can start mapping by following these commands
```
# source the workspace
source ~/rover_ws/install/setup.bash

# Launching Gazebo and controllers
ros2 launch rover launch_sim.launch.py world:=./src/rover/worlds/obstacles.world

# Launch the slam_toolbox
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/rover/config/mapper_params_online_async.yaml use_sim_time:=true

# Run rviz with the saved configuration
rviz2 -d src/rover/config/mapping.rviz 

# Run teleop_twist_keyboard to control the rover
ros2 run teleop_twist_keyboard teleop_twist_keyboard -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```


https://github.com/kalashjain23/rover/assets/97672680/b83ff8f5-b500-4774-99fc-8b627039eb75


## **Localization**

First, edit the [mapper_params_online_async.yaml](https://github.com/kalashjain23/rover/blob/d850479213e026ad81164b0ba3b702957188e3ad/config/mapper_params_online_async.yaml#LL17C23-L17C23) file to set the mode to localization and add the saved map.
```
    mode: localization  # Changed from mapping

    map_file_name: # path to your saved map

    map_start_at_dock: true
```
Now you can run the following commands to visualize.
```
# source the workspace
source ~/rover_ws/install/setup.bash

# Launching Gazebo and controllers
ros2 launch rover launch_sim.launch.py world:=./src/rover/worlds/obstacles.world

# Launch the slam_toolbox
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/rover/config/mapper_params_online_async.yaml use_sim_time:=true

# Run rviz with the saved configuration
rviz2 -d src/rover/config/slamming.rviz 

# Run teleop_twist_keyboard to control the rover
ros2 run teleop_twist_keyboard teleop_twist_keyboard -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```  
## **Navigation**
Run the following commands to setup your system for navigation
```
# source the workspace
cd ~/rover_ws && source ~/rover_ws/install/setup.bash

# Launching Gazebo and controllers
ros2 launch rover launch_sim.launch.py world:=./src/rover/worlds/obstacles.world

# Run Rviz2 with the saved configuration
rviz2 -d src/rover/config/navigation.rviz

# Launch the slam_toolbox
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/rover/config/mapper_params_online_async.yaml use_sim_time:=true

# Run twist_mux to publish data on the correct topic
ros2 run twist_mux twist_mux --ros-args --params-file ./src/rover/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped

# Launch Nav2 for navigation
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```  
*You can now start giving goal poses to the rover through Rviz2.*


https://github.com/kalashjain23/rover/assets/97672680/37563384-b494-4d4c-9429-9d98d0782e38

