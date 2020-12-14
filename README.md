# ASEN 5519 Final Project
## General
Henry Moore

12/13/20

Warehouse Robot Motion Planning

Videos can be found at https://drive.google.com/drive/folders/1pelyXJog8KlK1TazH1huYupifvMuwYna?usp=sharing

## Dependencies

Requires ROS Kinetic, gazebo, python 2.7, numpy, matplotlib, and all associated dependencies.

## Map Generation
To generate the map, see https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/. We use both the PGM and YAML file created by this in our software.

## Map loading and C-Space Construction
Run map_load.py for a demo of map loading and the associated C-Space construction. Must have generated a map of the desired workspace using turtlebot3 SLAM (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/). Map YAML and PGM files must be in program directory.

## Motion Planning
Run motion_planning.py to construct the c-space and generate motion plans between all defined goals. Saves generated goal matrix to 'goal_matrix.npy'.

## Motion Commanding

Must have a 'goal_matrix.npy' file. Must have brought up turtlebot3_gazebo (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/). Type goal number (goals are hard-coded in C-Space coordinate frame) to navigate from current goal to destination. Robot initial location must be able to move directly to goal 0.

