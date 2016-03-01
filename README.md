# myo_kuka


To switch the controller to Joint Impedance control just 
  - rosservice call /left_arm/controller_manager/switch_controller "{start_controllers: ['teleoperation_controller_mt_effort'], stop_controllers: ['joint_trajectory_controller'], strictness: 2}"
