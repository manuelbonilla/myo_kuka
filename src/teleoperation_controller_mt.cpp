
#include <teleoperation_controller_mt.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>

namespace myo_kuka
{
    TeleoperationControllerMT::TeleoperationControllerMT() {}
    TeleoperationControllerMT::~TeleoperationControllerMT() {}

    bool TeleoperationControllerMT::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
            return false;
        }

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

        q_cmd_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);

        //Desired posture is the current one
        x_des_ = x_;

        cmd_flag_ = 0;

        sub_command_ = nh_.subscribe("command1", 1, &TeleoperationControllerMT::command, this);
        sub_command_2 = nh_.subscribe("command2", 1, &TeleoperationControllerMT::command2, this);
        //sub_command_2 = nh_.subscribe("command2", 1, &TeleoperationControllerMT::command2, this);

        nh_.param<double>("alpha1", alpha1, 1);
        nh_.param<double>("alpha2", alpha2, 1);
        

        second_task = false;
        return true;
    }

    void TeleoperationControllerMT::starting(const ros::Time& time)
    {

    }

    void TeleoperationControllerMT::update(const ros::Time& time, const ros::Duration& period)
    {

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }

        if (cmd_flag_)
        {
            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

            // computing J_pinv_
            pseudo_inverse(J_.data, J_pinv_);

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

            // end-effector position error
            x_err_.vel = x_des_.p - x_.p;

            // getting quaternion from rotation matrix
            x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
            x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

            skew_symmetric(quat_des_.v, skew_);

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;
                for (int k = 0; k < skew_.cols(); k++)
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
            }

            // end-effector orientation error
            x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;

            // computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += alpha1 * J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
          
            }



            if (second_task)
            {
                Eigen::Matrix<double, 7, 7> P;
                P =  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv_*J_.data;
                KDL::Jacobian J_2;
                J_2.resize(kdl_chain_.getNrOfJoints());
                jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_2,4);

                // ROS_INFO_STREAM(J_2.data);
                Eigen::Matrix<double, 7,1> q_null;
                Eigen::MatrixXd J_pinv_2;
                pseudo_inverse(J_2.data, J_pinv_2);
                Eigen::Matrix<double, 7, 6> NullSpace = Eigen::Matrix<double, 7, 6>::Zero();
                NullSpace = P*J_pinv_2;
                KDL::Twist x_err_2;
                KDL::Frame x_2;
                fk_pos_solver_->JntToCart(joint_msr_states_.q,x_2, 4);
                x_err_2.vel = x_2.p - x_des_2.p;
                for (int i = 0; i < J_pinv_.rows(); i++)
                {
                    // joint_des_states_.qdot(i) = 0.0;
                    for (int k = 0; k < J_pinv_.cols(); k++)
                        joint_des_states_.qdot(i) += alpha2*NullSpace(i,k)*x_err_2(k); //removed scaling factor of .7
              
                }


            }



            // integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }

            if (Equal(x_, x_des_, 0.005))
            {
                ROS_INFO("On target");
                cmd_flag_ = 0;
            }
        }

        // set controls for joints
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }
    }

    void TeleoperationControllerMT::command(const geometry_msgs::Pose::ConstPtr &msg)
    {
        KDL::Frame frame_des_;


            frame_des_ = KDL::Frame(
                    KDL::Rotation::Quaternion(msg->orientation.x,
                                      msg->orientation.y,
                                      msg->orientation.z,
                                      msg->orientation.w),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));

        x_des_ = frame_des_;
        cmd_flag_ = 1;
    }

        void TeleoperationControllerMT::command2(const geometry_msgs::Pose::ConstPtr &msg)
    {
        KDL::Frame frame_des_;


            frame_des_ = KDL::Frame(
                    KDL::Rotation::Quaternion(msg->orientation.x,
                                      msg->orientation.y,
                                      msg->orientation.z,
                                      msg->orientation.w),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));

        x_des_2 = frame_des_;
        second_task =  true;
    }


 /*       void TeleoperationControllerMT::command2(const geometry_msgs::Pose::ConstPtr &msg)
    {
        KDL::Frame frame_des_2;


            frame_des_2 = KDL::Frame(
                    KDL::Rotation::Quaternion(msg->orientation.x,
                                      msg->orientation.y,
                                      msg->orientation.z,
                                      msg->orientation.w),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));

        x_des_2 = frame_des_2;
        cmd_flag_ = 1;
    }*/
}

PLUGINLIB_EXPORT_CLASS(myo_kuka::TeleoperationControllerMT, controller_interface::ControllerBase)
