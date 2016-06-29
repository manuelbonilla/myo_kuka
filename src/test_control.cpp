#include <pluginlib/class_list_macros.h>
#include <math.h>

#include "teleoperation_controllers/test_control.h"

namespace teleoperation_controllers 
{
    test_control::test_control() {}
    test_control::~test_control() {}
    
    bool test_control::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
        
        // find stiffness (dummy) joints; this is necessary until proper position/stiffness/damping interface exists
        // for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        // {
        //     joint_stiffness_handles_.push_back(robot->getHandle(it->getJoint().getName()+"_stiffness"));
        // }
        
        // ROS_DEBUG("found %lu stiffness handles", joint_stiffness_handles_.size());
        
        // previous_stiffness_.resize(joint_stiffness_handles_.size());
        
        return true;
    }
    
    void test_control::starting(const ros::Time& time)
    {
        // for(size_t i=0; i<joint_handles_.size(); i++)
        // {
        //    previous_stiffness_[i] = joint_stiffness_handles_[i].getPosition();
        // }
    }
    
    void test_control::update(const ros::Time& time, const ros::Duration& period)
    {
        // update the commanded position to the actual, so that the robot doesn't 
        // go back at full speed to the last commanded position when the stiffness 
        // is raised again
        for(size_t i=0; i<joint_handles_.size(); i++) 
        {
            //joint_handles_[i].setCommand(joint_handles_[i].getPosition());
            joint_handles_[i].setCommand( 0.0 );
            // joint_stiffness_handles_[i].setCommand(DEFAULT_STIFFNESS);
        }
    }
    
    void test_control::stopping(const ros::Time& time)
    {
        //for(size_t i=0; i<joint_handles_.size(); i++)
        //{
        //    joint_stiffness_handles_[i].setCommand(previous_stiffness_[i]);
        //}
    }

}

PLUGINLIB_EXPORT_CLASS(teleoperation_controllers::test_control, controller_interface::ControllerBase)
