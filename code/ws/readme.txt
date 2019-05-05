roscore
roslaunch ll4ma_robots_gazebo lbr4.launch end_effector:=allegro robot_table:=true world_launch_file:=grasp_scene.launch
python src/ll4ma_robots_gazebo/scripts/data_grabber.py -n 10000 -m 10000 -r 256 256
