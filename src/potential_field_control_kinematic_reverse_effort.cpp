#include <potential_field_control_kinematic_reverse_effort.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf_conversions/tf_kdl.h>

#include <math.h>


namespace myo_kuka
{
PotentialFieldControlKinematicReverseEffort::PotentialFieldControlKinematicReverseEffort() {}
PotentialFieldControlKinematicReverseEffort::~PotentialFieldControlKinematicReverseEffort() {}

bool PotentialFieldControlKinematicReverseEffort::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
  ROS_INFO("Starting controller");
  ROS_WARN("Number of segments: %d", kdl_chain_.getNrOfSegments());

  parameters_.k_p = Eigen::Matrix<double, 6, 6>::Zero();
  parameters_.k_d = Eigen::Matrix<double, 6, 6>::Zero();
  parameters_.k_i = Eigen::Matrix<double, 6, 6>::Zero();

// for switch the hand_desired
  start_controller = false;
  load_parameters(n);

//resize the vector that we use for calculates the dynamic
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  J_.resize(kdl_chain_.getNrOfJoints());
  joint_des_states_filtered.resize(kdl_chain_.getNrOfJoints());
  joint_des_states_old.resize(kdl_chain_.getNrOfJoints());


  F_repulsive = Eigen::Matrix<double, 6, 1>::Zero();
  F_attractive = Eigen::Matrix<double, 6, 1>::Zero();
  F_total = Eigen::Matrix<double, 6, 1>::Zero();


  ROS_INFO("Subscribed for desired_reference to: %s", "command");
  sub_command = nh_.subscribe(topic_desired_reference.c_str(), 1, &PotentialFieldControlKinematicReverseEffort::command1, this);
//list of obstacles
  ROS_INFO("Subscribed for obstacle_avoidance_topic to : %s", topic_obstacle_avoidance.c_str());
  sub_obstacles = nh_.subscribe(topic_obstacle_avoidance.c_str(), 1, &PotentialFieldControlKinematicReverseEffort::InfoGeometry, this);

  sub_start_controller = nh_.subscribe("start_controller", 1, &PotentialFieldControlKinematicReverseEffort::startControllerCallBack, this);


//callcback for setting the gains at real time

  pub_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
  pub_error_id = nh_.advertise<desperate_housewife::Error_msg>("error_id", 1000);
  pub_q = nh_.advertise<std_msgs::Float64MultiArray>("q_commad", 1000);
  pub_qp = nh_.advertise<std_msgs::Float64MultiArray>("qp_commad", 1000);
  pub_pf_attractive_force = nh_.advertise<std_msgs::Float64MultiArray>("F_attractive", 1000);
  pub_pf_repulsive_forse = nh_.advertise<std_msgs::Float64MultiArray>("F_repulsive", 1000);
  pub_pf_total_force = nh_.advertise<std_msgs::Float64MultiArray>("F_total", 1000);

  pub_total_wrench = nh_.advertise<geometry_msgs::WrenchStamped>("total_end_effector_wrench", 512);

  srv_start_controller = nh_.advertiseService("load_parameters", &PotentialFieldControlKinematicReverseEffort::loadParametersCallback, this);
  srv_load_velocity = nh_.advertiseService("load_velocity", &PotentialFieldControlKinematicReverseEffort::loadVelocityCallback, this);

  collisions_lines_pub = nh_.advertise<visualization_msgs::MarkerArray>("collision_lines", 1);
  arrows_pub = nh_.advertise<visualization_msgs::MarkerArray>("collision_arrows", 1);

  error_id.id = 10000;
// error_id.id_arm = parameters_.id_arm;
  return true;
}

void PotentialFieldControlKinematicReverseEffort::starting(const ros::Time& time)
{

  for (unsigned int i = 0; i < joint_handles_.size(); i++)
  {
    joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    joint_des_states_.q(i) = joint_msr_states_.q(i);
    joint_des_states_old.qdot(i) = 0.0;
  }

  fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);
  x_now_int = x_des_;
  x_des_int = x_des_;

  SetToZero(x_err_integral);
  SetToZero(x_err_);
  x_err_.vel.data[0] = 10000.0;
  start_controller = false;
}

void PotentialFieldControlKinematicReverseEffort::update(const ros::Time& time, const ros::Duration& period)
{

  std_msgs::Float64MultiArray q_msg;
  std_msgs::Float64MultiArray qp_msg;
  std_msgs::Float64MultiArray F_repulsive_msg;
  std_msgs::Float64MultiArray F_attractive_msg;
  std_msgs::Float64MultiArray F_total_msg;
  std_msgs::Float64MultiArray err_msg;

  for (unsigned int i = 0; i < joint_handles_.size(); i++)
  {
    joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
  }

  KDL::JntArray vel_repulsive;
  vel_repulsive.resize(7);
  SetToZero(vel_repulsive);

  tf::twistKDLToMsg(x_err_, error_id.error_);

// flag to use this code with real robot

  KDL::Twist x_err_msg;
  x_err_msg = x_err_;

  if (start_controller)
  {

    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

// computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);


    if (parameters_.enable_interpolation)
    {
      x_des_.p = x_now_int.p + interpolatormb(time_inter, parameters_.max_time_interpolation) * (x_des_int.p - x_now_int.p);

      x_des_int.M.GetQuaternion(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);
      x_now_int.M.GetQuaternion(quat_now.v(0), quat_now.v(1), quat_now.v(2), quat_now.a);

      tf::Quaternion quat_tf_des_int(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);
      tf::Quaternion quat_tf_now_int(quat_now.v(0), quat_now.v(1), quat_now.v(2), quat_now.a);
      quat_tf = (tf::slerp(quat_tf_now_int, quat_tf_des_int, Time)).normalize();
      tf::quaternionTFToKDL(quat_tf, x_des_.M);

      KDL::Twist x_err_int;


      time_inter = time_inter + period.toSec();

// SO3 Time update
      Time = interpolatormb(time_inter, parameters_.max_time_interpolation);
    }
    else
    {
      x_des_ = x_des_int;
    }

    x_dot_ = J_.data * joint_msr_states_.qdot.data;

    x_err_ = diff(x_, x_des_);

    F_total = Eigen::Matrix<double, 6, 1>::Zero();
    F_repulsive  = Eigen::Matrix<double, 6, 1>::Zero();

    if (parameters_.enable_obstacle_avoidance)
    {
      lines_total.markers.clear();
      arrows_total.markers.clear();
      id_global = 0;
      double num_of_links_in_potential = 0.0;
      Eigen::Matrix<double, 6, 1> F_obj_base_link = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 6, 1> F_table_base_link = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 6, 1> F_obj_base_total = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 6, 1> F_table_base_total = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 6, 1> F_table = Eigen::Matrix<double, 6, 1>::Zero();

      for (unsigned int i = 0; i < parameters_.pf_list_of_links.size(); ++i)
      {
        KDL::JntArray joint_states_chain(parameters_.pf_list_of_chains[i].getNrOfJoints());

        for (unsigned int j = 0; j < parameters_.pf_list_of_chains[i].getNrOfJoints(); ++j)
        {
          joint_states_chain(j) = joint_msr_states_.q(j);
        }

        KDL::Frame fk_chain;
        parameters_.pf_list_of_fk[i].JntToCart(joint_states_chain, fk_chain);
        Eigen::Matrix<double, 6, 6> Adjoint;
        Adjoint = getAdjointT( fk_chain );
        Eigen::Matrix<double, 6, 1> F_obj = Eigen::Matrix<double, 6, 1>::Zero();

        for (unsigned int k = 0; k < Object_position.size(); ++k)
        {
          double influence_local = parameters_.pf_dist_to_obstacles;
          Eigen::Matrix<double, 6, 1> F_obj_local = Eigen::Matrix<double, 6, 1>::Zero();
          F_obj_local = getRepulsiveForceObjects(fk_chain, influence_local, Object_position[k], Object_radius[k], Object_height[k] );
          F_obj = F_obj + F_obj_local;
          if (F_obj_local.norm() != 0.0)
          {
            num_of_links_in_potential = num_of_links_in_potential + 1.0;
          }
          ++id_global;
        }

        F_obj_base_link = Adjoint * F_obj;

        F_obj_base_total += F_obj_base_link;



        F_table = getRepulsiveForceTable(fk_chain, parameters_.pf_dist_to_table, parameters_.pf_repulsive_gain_table );
        if (F_table.norm() != 0.0)
        {
          num_of_links_in_potential = num_of_links_in_potential + 1.0;
        }
        F_table_base_link = Adjoint * F_table;
        F_table_base_total += F_table_base_link;

        vel_repulsive.data += getRepulsiveJointVelocity( J_, parameters_.pf_list_of_chains[i].getNrOfJoints(), (F_table_base_link + F_obj_base_link) );
        collisions_lines_pub.publish(lines_total);
        arrows_pub.publish(arrows_total);


      }


      F_repulsive  = (F_table_base_total + F_obj_base_total);

      if (num_of_links_in_potential > 1.0)
      {
        vel_repulsive.data = (1.0 / num_of_links_in_potential) * vel_repulsive.data;
      }

    }

    if (parameters_.enable_attractive_field)
    {
      KDL::Twist x_dot_d;

      x_dot_d.vel.data[0] = parameters_.k_p(0, 0) / parameters_.k_d(0, 0) * x_err_.vel.data[0];
      x_dot_d.vel.data[1] = parameters_.k_p(1, 1) / parameters_.k_d(1, 1) * x_err_.vel.data[1];
      x_dot_d.vel.data[2] = parameters_.k_p(2, 2) / parameters_.k_d(2, 2) * x_err_.vel.data[2];

      x_dot_d.rot.data[0] = parameters_.k_p(3, 3) / parameters_.k_d(3, 3) * x_err_.rot.data[0];
      x_dot_d.rot.data[1] = parameters_.k_p(4, 4) / parameters_.k_d(4, 4) * x_err_.rot.data[1];
      x_dot_d.rot.data[2] = parameters_.k_p(5, 5) / parameters_.k_d(5, 5) * x_err_.rot.data[2];
      // ROS_INFO_STREAM("x_err_.rot : " << x_err_.rot.data[0] << " " << x_err_.rot.data[1] << " " << x_err_.rot.data[2]);
      // ROS_INFO_STREAM("x_dot_d.rot : " << x_dot_d.rot.data[0] << " " << x_dot_d.rot.data[1] << " " << x_dot_d.rot.data[2]);

      double v_limited = VelocityLimit(x_dot_d, parameters_.vel_limit_robot);

      x_err_integral += x_err_ * period.toSec();
      for (int i = 0; i < F_attractive.size(); i++)
      {
        F_attractive(i) =  -parameters_.k_d(i, i) * ( x_dot_(i) -  v_limited * x_dot_d(i) ) + parameters_.k_i(i, i) * x_err_integral(i);
        // ROS_INFO_STREAM("F_attractive: " << F_attractive.transpose());
// F_attractive(i) =  parameters_.k_p(i, i) * x_err_(i);
      }
    }

    Eigen::MatrixXd x_err_eigen_ = Eigen::MatrixXd::Zero(6, 1);

    x_err_eigen_ = F_attractive;

    int n_task = 2;
    std::vector<Eigen::MatrixXd> qp(n_task + 1, Eigen::MatrixXd::Zero(7, 1));
    qp[n_task] = Eigen::MatrixXd::Zero(7, 1);

    std::vector<Eigen::MatrixXd> xp(n_task + 1);

    // Eigen::MatrixXd secondTask = CF_JS_PotentialEnergy( joint_msr_states_.q );
    KDL::JntArray G_local(7);
    id_solver_->JntToGravity(joint_msr_states_.q, G_local);
    Eigen::MatrixXd secondTask = parameters_.gain_null_space * G_local.data;



/************************/
    // KDL::Frame x_2;
    // fk_pos_solver_->JntToCart(joint_msr_states_.q, x_2, 6);


    // KDL::Jacobian J_2;
    // J_2.resize(kdl_chain_.getNrOfJoints());
    // jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_2, 6);
    // KDL::Twist x_err_2;
    // x_err_2 = diff(x_2, x_des_2);
    // Eigen::MatrixXd J_pinv_2;
    // pseudo_inverse(J_2.data, J_pinv_2);

    // Eigen::MatrixXd x_err_2_eigen=Eigen::MatrixXd::Zero(6,1);
    // x_err_eigen_ << x_err_2.vel(0), x_err_2.vel(1), x_err_2.vel(2), x_err_2.rot(0), x_err_2.rot(1), x_err_2.rot(2);

    // secondTask = parameters_.gain_null_space * J_pinv_2 * x_err_eigen_; 

/*******************************/


    xp[0] = x_err_eigen_;
    xp[1] = secondTask;

    std::vector<Eigen::MatrixXd> J(n_task + 1);
    J[0] = J_.data;
    J[1] = Eigen::MatrixXd::Identity(7, 7);

    qp[1] = secondTask;

    Eigen::MatrixXd Pp1 = Eigen::MatrixXd::Identity(7, 7);
    for (int i = n_task - 1 ; i >= 0; i--)
    {
      Eigen::MatrixXd JkPkp_pseudo;
      pseudo_inverse(J[i] * Pp1,  JkPkp_pseudo);
      qp[i] = qp[i + 1] + JkPkp_pseudo * (xp[i] - J[i] * qp[i + 1]);
    }

    for (int i = 0; i < 7; i++)
    {
      joint_des_states_.qdot(i) = 0.0;
      joint_des_states_.qdot(i) +=  qp[0](i);
    }

    for (int i = 0; i < 7; i++)
    {
// joint_des_states_filtered.qdot(i) =  filters::exponentialSmoothing(joint_des_states_.qdot(i), joint_des_states_old.qdot(i), 0.5);
      joint_des_states_filtered.qdot(i) = joint_des_states_.qdot(i);
      joint_des_states_old.qdot(i) = joint_des_states_filtered.qdot(i);
    }

    if (parameters_.enable_obstacle_avoidance)
    {
      for (int i = 0; i < 7; i++)
      {
        joint_des_states_filtered.qdot(i) += vel_repulsive.data[i] ;
      }
    }

    saturateJointVelocities( joint_des_states_filtered.qdot, parameters_.max_vel_percentage);

    x_err_msg = diff(x_, x_des_int);

    tf::twistKDLToMsg(x_err_msg, error_id.error_);

    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
      if ( !checkStatesforNan(joint_des_states_filtered.qdot) )
      {
        joint_des_states_.q(i) += period.toSec() * joint_des_states_filtered.qdot(i);
      }
    }


// for (unsigned i = 0; i < joint_des_states_filtered.qdot.rows(); ++i) {
//     ROS_INFO_STREAM("JV: " << joint_des_states_filtered.qdot.data.transpose() * 180.0 / M_PI);
// }

    saturateJointPositions( joint_des_states_.q, 1.0 * M_PI / 180.0 );


  }

  pub_error_id.publish( error_id );

  for (unsigned int i = 0; i < F_total.size(); i++ )
  {
    F_repulsive_msg.data.push_back(F_repulsive(i));
    F_attractive_msg.data.push_back(F_attractive(i));
    F_total_msg.data.push_back(F_total(i));
    err_msg.data.push_back(x_err_msg(i));
  }

  for (unsigned int i = 0; i < joint_handles_.size(); i++)
  {
    joint_handles_[i].setCommand(joint_des_states_.q(i));
    q_msg.data.push_back(joint_des_states_.q(i));
    qp_msg.data.push_back(joint_des_states_filtered.qdot(i));
  }


  pub_pf_repulsive_forse.publish(F_repulsive_msg);
  pub_pf_attractive_force.publish(F_attractive_msg);
  pub_pf_total_force.publish(F_total_msg);
  pub_error.publish(err_msg);
  pub_q.publish(q_msg);
  pub_qp.publish(qp_msg);

  ros::spinOnce();
}

void PotentialFieldControlKinematicReverseEffort::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
{
  KDL::Frame frame_des_;
  tf::poseMsgToKDL(msg->pose, frame_des_);


  if (!Equal(frame_des_, x_des_int, 0.05))
  {
// update desired frame;
// F_attractive_last = F_attractive;
    x_des_int = frame_des_;
// update robot position
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_now_int);
// time update
    time_inter = 0;
    Time = 0;
// switch_trajectory = true;
    x_err_last = x_err_;
    SetToZero(x_err_integral);
  }

  error_id.id = msg->id;
}

void PotentialFieldControlKinematicReverseEffort::command1(const geometry_msgs::Pose::ConstPtr &msg)
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

  x_des_int = frame_des_;

  tf::Transform CollisionTransform;
  tf::transformKDLToTF( x_des_int, CollisionTransform);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( CollisionTransform, ros::Time::now(), "world", "reference") );

  // ROS_INFO_STREAM("New reference");
  // cmd_flag_ = 1;
}


void PotentialFieldControlKinematicReverseEffort::command2(const geometry_msgs::Pose::ConstPtr &msg)
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
    // second_task =  true;


    tf::Transform CollisionTransform2;
    tf::transformKDLToTF( frame_des_, CollisionTransform2);
    elbow_reference.sendTransform( tf::StampedTransform( CollisionTransform2, ros::Time::now(), "world", "elbow_reference") );
}


void PotentialFieldControlKinematicReverseEffort::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  Object_radius.clear();
  Object_height.clear();
  Object_position.clear();

  for (unsigned int i = 0; i < msg->geometries.size(); i++)
  {
    KDL::Frame frame_obj;
    Object_radius.push_back(msg->geometries[i].info[0]);
    Object_height.push_back(msg->geometries[i].info[1]);
    tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
    Object_position.push_back(frame_obj);
  }

  ROS_INFO("New environment: No of Obstacles %lu", msg->geometries.size());
}




void PotentialFieldControlKinematicReverseEffort::load_parameters(ros::NodeHandle &n)
{
  nh_.param<std::string>("topic_obstacle", topic_obstacle_avoidance, "obstacles");
  nh_.param<std::string>("tip_name", parameters_.tip_name, "end_effector");
  nh_.param<std::string>("root_name", parameters_.root_name, "world");
  nh_.param<std::string>("topic_desired_reference", topic_desired_reference, "command");
  nh_.param<double>("time_interpolation", parameters_.max_time_interpolation, 2.0);
  nh_.param<double>("max_vel_percentage", parameters_.max_vel_percentage, 0.5);
  nh_.param<double>("pf_repulsive_gain_obstacles", parameters_.pf_repulsive_gain_obstacles , 1.0);
  nh_.param<double>("pf_repulsive_gain_table", parameters_.pf_repulsive_gain_table , 1.0);
  nh_.param<double>("pf_dist_to_obstacles", parameters_.pf_dist_to_obstacles , 1.0);
  nh_.param<double>("pf_dist_to_table", parameters_.pf_dist_to_table , 1);
  nh_.param<double>("vel_limit_robot", parameters_.vel_limit_robot , 0.5);
  nh_.param<double>("gain_null_space", parameters_.gain_null_space , 0.2);

  nh_.param<bool>("enable_obstacle_avoidance", parameters_.enable_obstacle_avoidance , true);
  nh_.param<bool>("enable_joint_limits_avoidance", parameters_.enable_joint_limits_avoidance , true);
  nh_.param<bool>("enable_attractive_field", parameters_.enable_attractive_field , true);
  nh_.param<bool>("enable_null_space", parameters_.enable_null_space , true);
  nh_.param<bool>("enable_interpolation", parameters_.enable_interpolation , false);
  nh_.param<int>("id_arm", parameters_.id_arm , 0);
// ROS_INFO("topic_desired_reference: %s", desired_reference_topic.c_str());
  ROS_INFO("topic_obstacle: %s", topic_obstacle_avoidance.c_str());
  ROS_INFO("link_tip_name: %s", parameters_.tip_name.c_str());
  ROS_INFO("link_root_name: %s", parameters_.root_name.c_str());
  ROS_INFO("time_interpolation: %f", parameters_.max_time_interpolation);
  ROS_INFO("max_vel_percentage: %f", parameters_.max_vel_percentage);
  ROS_INFO("pf_repulsive_gain_obstacles: %f", parameters_.pf_repulsive_gain_obstacles);
  ROS_INFO("pf_repulsive_gain_table: %f", parameters_.pf_repulsive_gain_table);
  ROS_INFO("pf_dist_to_obstacles: %f", parameters_.pf_dist_to_obstacles);
  ROS_INFO("pf_dist_to_table: %f", parameters_.pf_dist_to_table);
  ROS_INFO("vel_limit_robot: %f", parameters_.vel_limit_robot);
  ROS_INFO("gain_null_space: %f", parameters_.gain_null_space);

  ROS_INFO_STREAM("Obstacle avoidance " << (parameters_.enable_obstacle_avoidance ? "Enabled" : "Disabled") );
  ROS_INFO_STREAM("Joint Limit Avoidance: " << (parameters_.enable_joint_limits_avoidance ? "Enabled" : "Disabled") );
  ROS_INFO_STREAM("Attractive Field: " << (parameters_.enable_attractive_field ? "Enabled" : "Disabled") );
  ROS_INFO_STREAM("Null Space: " << (parameters_.enable_null_space ? "Enabled" : "Disabled") );
  ROS_INFO_STREAM("Interpolation: " << (parameters_.enable_interpolation ? "Enabled" : "Disabled") );

  ROS_INFO_STREAM("start_controller: " << (start_controller ? "Enabled" : "Disabled") );

  if (!start_controller)
  {
    XmlRpc::XmlRpcValue my_list;
    nh_.getParam("links_with_potential_field", my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    parameters_.pf_list_of_links.clear();
    parameters_.pf_list_of_chains.clear();
    parameters_.pf_list_of_fk.clear();

    for ( int i = 0; i < my_list.size(); ++i)
    {
      ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      parameters_.pf_list_of_links.push_back(static_cast<std::string>(my_list[i]).c_str());
      ROS_INFO("Link %s defined as collision point", static_cast<std::string>(my_list[i]).c_str());
    }

    for (unsigned int i = 0; i < parameters_.pf_list_of_links.size(); ++i)
    {
      KDL::Chain chain_tmp;

      if (!kdl_tree_.getChain(parameters_.root_name.c_str(), parameters_.pf_list_of_links[i].c_str(), chain_tmp))
      {
        ROS_ERROR("Error processing chain from %s to %s ", parameters_.root_name.c_str(), parameters_.pf_list_of_links[i].c_str());
      }
      else
      {
        parameters_.pf_list_of_chains.push_back(chain_tmp);
        parameters_.pf_list_of_fk.push_back(KDL::ChainFkSolverPos_recursive(chain_tmp));
        parameters_.pf_list_of_jac.push_back(KDL::ChainJntToJacSolver(chain_tmp));
        ROS_INFO("Chain from %s to %s correctly initialized. Num oj Joints = %d", parameters_.root_name.c_str(), parameters_.pf_list_of_links[i].c_str(),
                 chain_tmp.getNrOfJoints());
      }
    }
  }

  parameters_.k_p = getGainMatrix(std::string("k_p"), n, 6);
  parameters_.k_d = getGainMatrix(std::string("k_d"), n, 6);
  parameters_.k_i = getGainMatrix(std::string("k_i"), n, 6);

  ROS_INFO_STREAM("k_p" << std::endl << parameters_.k_p);
  ROS_INFO_STREAM("k_d" << std::endl << parameters_.k_d);
  ROS_INFO_STREAM("k_i" << std::endl << parameters_.k_i);

  ROS_INFO("Parameters loaded");
  return;
}


void PotentialFieldControlKinematicReverseEffort::startControllerCallBack(const std_msgs::Bool::ConstPtr& msg)
{
  start_controller = msg->data;
 ROS_INFO_STREAM("start_controller: " << (start_controller ? "Enabled" : "Disabled") );
  return;
}


bool PotentialFieldControlKinematicReverseEffort::loadParametersCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  load_parameters(nh_);
  return true;
}

bool PotentialFieldControlKinematicReverseEffort::loadVelocityCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  loadVelocity();
  return true;
}


void PotentialFieldControlKinematicReverseEffort::loadVelocity()
{
  nh_.param<double>("max_vel_percentage", parameters_.max_vel_percentage, 0.5);
  nh_.param<double>("vel_limit_robot", parameters_.vel_limit_robot , 0.5);

  if (parameters_.max_vel_percentage > 0.95)
  {
    parameters_.max_vel_percentage = 0.95;
  }
  if (parameters_.vel_limit_robot > 4.0)
  {
    parameters_.vel_limit_robot = 4.0;
  }
  if (parameters_.max_vel_percentage < 0.5)
  {
    parameters_.max_vel_percentage = 0.5;
  }
  if (parameters_.vel_limit_robot < 1.0)
  {
    parameters_.vel_limit_robot = 1.0;
  }

  nh_.setParam("max_vel_percentage", parameters_.max_vel_percentage);
  nh_.setParam("vel_limit_robot", parameters_.vel_limit_robot);

  ROS_INFO("max_vel_percentage: %f", parameters_.max_vel_percentage);
  ROS_INFO("vel_limit_robot: %f", parameters_.vel_limit_robot);
}

}



PLUGINLIB_EXPORT_CLASS(myo_kuka::PotentialFieldControlKinematicReverseEffort, controller_interface::ControllerBase)

