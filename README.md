# Devon Tour Guide
A Turtlebot robot guide that features point-to-point (P2P) and tour guide capabilities in the University of Oklahomas's Devon Energy Hall (DEH).

# Pre-requisites

The tour options use ROS's navigation stack, however there is a certain
configuration to the ROS default files you need to modify:

- The file to modify is
  `/opt/ros/kinetic/share/turtlebot_navigation/param/dwa_local_planner_params.yaml`
- To change directory to that using ROS's native functions, use `roscd
  turtlebot_navigation/param`

The repo's main directory has a modified .yaml file (called
`modified_dwa_local_planner_params.yaml`). When replacing these files, be sure
to create a backup.

# How to Run

The launch file will execute three ROS commands and a Python script that will
serve the main functions of the P2P guide and Tour guide. These commands are
(in order):

- `roslaunch turtlebot_gazebo turtlebot_world.launch
  world_file:=~/$PATH_TO/devon_tour_guide/worlds/devon1st.world`
- `roslaunch turtlebot_gazebo amcl_demo.launch
  map_file:=/$PATH_TO/devon_tour_guide/buildings/devon_v2.yaml`
- `roslaunch turtlebot_rviz_launchers view_navigation.launch` 
  - *Note: this requires you to manually set the initial 2D pose before running
    the script*
- `python /$PATH_TO/devon_tour_guide/scripts/main.py`

*If the launch file does not work/is missing:* running the listed commands/script will
suffice 

# Point-to-Point Guide

The P2P tour option (when user initially enters '1') will list a few options
for the turtlebot to move to. Each option will send the robot to that
highlight's coordinates and print if the navigation was successful or not.

# Tour Guide

The tour guide option (when user initially enters '2') takes the turtlebot to the nearest
entrance (either West or East) and begins moving to each nearest highlight. 
