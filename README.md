# myo_kuka

To run the demo in simulation

  - roslaunch myo_kuka run_vito.launch

Start Matlab

  - rostopic pub - /right_arm/teleoperation_controller_mt/start_controller std_msgs/Bool True

To star demo in real

  - roslaunch myo_kuka run_vito.launch use_robot_sim:=false right_arm_enabled:=true

  - rosservice call /right_arm/controller_manager/switch_controller "{start_controllers: ['teleoperation_controller_mt_effort'], stop_controllers: ['teleoperation_controller_mt'], strictness: 2}"

Start Matlab

  - rostopic pub - /right_arm/teleoperation_controller_mt_effort/start_controller std_msgs/Bool True


To test the controller publish a reference in topics command1 or command2. For example

  - rostopic pub -1 /right_arm/teleoperation_controller_mt/command1 geometry_msgs/Pose "position:
  x: -1.0
  y: 0.7
  z: 0.4
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0" 

  - rostopic pub  -1 /right_arm/teleoperation_controller_mt/command2 geometry_msgs/Pose "position:
  x: -0.3
  y: -0.7
  z: 0.2
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0" 


