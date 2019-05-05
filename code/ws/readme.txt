roscore
roslaunch ll4ma_robots_gazebo lbr4.launch end_effector:=allegro robot_table:=true world_launch_file:=grasp_scene.launch
roslaunch lbr4_allegro_moveit_config  lbr4_allegro_moveit.launch
