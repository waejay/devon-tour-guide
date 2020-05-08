# Devon Tour Guide
A Turtlebot robot guide that features point-to-point (P2P) and tour guide capabilities in the University of Oklahomas's Devon Energy Hall (DEH).

# How to Run

The launch file will execute three ROS commands and a Python script that will
serve the main functions of the P2P guide and Tour guide. These commands are
(in order):

- `roslaunch turtlebot_gazebo turtlebot_world.launch
  world_file:=~/$PATH_TO/devon_tour_guide/worlds`
- `roslaunch turtlebot_gazebo amcl_demo.launch
  map_file:=/$PATH_TO/devon_tour_guide/buildings/devon_v2.yaml`
- `roslaunch turtlebot_rviz_launchers view_navigation.launch` 
  - *Note: this requires you to manually set the initial 2D pose before running
    the script*
- `py /$PATH_TO/devon_tour_guide/scripts/main.py`

*If the launch file does not work/is missing:* running the listed commands/script will
suffice 
