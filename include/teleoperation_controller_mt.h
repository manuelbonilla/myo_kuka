#ifndef TELEOPERATION_CONTROLLERMT_H
#define TELEOPERATION_CONTROLLERMT_H


#include <lwr_controllers/KinematicChainControllerBase.h>
#include <lwr_controllers/PoseRPY.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>

namespace myo_kuka
{
	class TeleoperationControllerMT: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
	public:
		TeleoperationControllerMT();
		~TeleoperationControllerMT();

		bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const geometry_msgs::Pose::ConstPtr &msg);
		void command2(const geometry_msgs::Pose::ConstPtr &msg);

	private:
		ros::Subscriber sub_command_, sub_command_2;
		ros::Subscriber sub_gains_;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_, x_des_2;	//desired pose

		KDL::Twist x_err_;

		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;
		Eigen::Matrix<double,3,3> skew_;
		double alpha1, alpha2;

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;
		
		int cmd_flag_;

		bool second_task;
		
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;

		ros::Publisher pub_error;
	};

}

#endif
