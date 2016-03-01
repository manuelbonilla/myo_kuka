# myo_kuka


To switch the controller to Joint Impedance control just 
  - rosservice call /right_arm/controller_manager/switch_controller "{start_controllers: ['teleoperation_controller_mt_effort'], stop_controllers: ['teleoperation_controller_mt'], strictness: 2}"

To test the controller publish a reference in topics command1 or command2. For example

  - rostopic pub /right_arm/teleoperation_controller_mt/command1 geometry_msgs/Pose "position:
  x: -1.0
  y: 0.7
  z: 0.4
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0" 

