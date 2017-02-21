
start teleoperation_controller_mt_effort

rosservice call /right_arm/controller_manager/switch_controller "{start_controllers: ['teleoperation_controller_mt_effort'], stop_controllers: ['joint_trajectory_controller','stiffness_trajectory_controller','damping_trajectory_controller','add_torque_trajectory_controller' ], strictness: 2}"


rosservice call /left_arm/controller_manager/switch_controller "{start_controllers: ['teleoperation_controller_mt_effort'], stop_controllers: ['joint_trajectory_controller','stiffness_trajectory_controller','damping_trajectory_controller','add_torque_trajectory_controller' ], strictness: 2}"

run controller

rostopic pub -1 /right_arm/teleoperation_controller_mt_effort/start_controller std_msgs/Bool True
rostopic pub -1 /left_arm/teleoperation_controller_mt_effort/start_controller std_msgs/Bool True



start home position

rosservice call /right_arm/controller_manager/switch_controller "{start_controllers: ['joint_trajectory_controller'], stop_controllers: ['teleoperation_controller_mt_effort','stiffness_trajectory_controller','damping_trajectory_controller','add_torque_trajectory_controller' ], strictness: 2}"