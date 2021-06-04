search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=panda_arm.srdf
robot_name_in_srdf=panda
moveit_config_pkg=panda_moveit_config
robot_name=panda_arm
planning_group_name=panda_arm
ikfast_plugin_pkg=panda_arm_ikfast_panda_arm_plugin
base_link_name=panda_link0
eef_link_name=panda_link8
ikfast_output_path=/home/hp/graspvgn_ws/src/panda_arm_ikfast_panda_arm_plugin/src/panda_arm_panda_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
