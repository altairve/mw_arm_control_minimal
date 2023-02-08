#include "mw_arm_control_minimal/orangewood_hardware_interface.h"

namespace mw_manipulation
{
  OrangewoodInterface::OrangewoodInterface(rclcpp::Node::SharedPtr node_ptr) : node_(node_ptr)
  {
    service_callback_group_ =
      node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    inner_callback_group_ =
      node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    execute_trajectory_service_ =
      node_->create_service<std_srvs::srv::Trigger>("/execute_trajectory",std::bind(&OrangewoodInterface::executeTrajectoryCallback,this,std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,service_callback_group_);
    stop_execution_service_ =
      node_->create_service<std_srvs::srv::SetBool>("/stop_trajectory_execution",std::bind(&OrangewoodInterface::stopExecutuionCallback,this,std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,service_callback_group_);

    joint_traj_action_client_ =
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_,"/joint_trajectory_controller/follow_joint_trajectory");



  }

  float OrangewoodInterface::betterStof(std::string &fs)
	{
  	float result;
  	try
  	{
    	result = std::stof(fs);
  	}
  	catch(const std::out_of_range& e){
    	std::cerr<<"string causing problem :"<<result<<std::endl;
  	}
  	return(result);
	}

  void OrangewoodInterface::readTrajectorymsgFromFile(moveit_msgs::msg::RobotTrajectory &trajectory,std::string path)
	{
    std::vector<std::string> joint_names = {"base","shoulder","link1","elbow","link2","w2","w3"};

    trajectory.joint_trajectory.joint_names = joint_names;
    trajectory.joint_trajectory.header.frame_id = joint_names[0];

		trajectory.joint_trajectory.points.clear();
		std::fstream readTrajectory;
		readTrajectory.open(path,std::fstream::in);
		if (!readTrajectory.is_open()) {
        std::cerr << "Could not open the file - '"
             << path << "'" << std::endl;
        return;
    }
    int counter = 0;
    int pointsreached = 0;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    float val;
		std::string line;
    while (getline(readTrajectory,line))
    {
      std::string fs(line);
      float val_float;
      long int val_int;

      if (counter != 12 && counter != 13)
        val_float = betterStof(fs);

      for (int i = 0; i < 6; i++)
      {
        if (counter == i)
        {
          point.positions.push_back(val_float);

        }
      }
      for (int i = 6; i < 12; i++)
      {
        if (counter == i)
        {
          point.velocities.push_back(val_float);

        }
      }

      if (counter == 12)
      {
        val_int = betterStof(fs);
        point.time_from_start.sec = val_int;
      }

      if (counter == 13)
      {
        val_int = betterStof(fs);
        point.time_from_start.nanosec = val_int;
        trajectory.joint_trajectory.points.push_back(point);
      }
      counter++;
      if (counter == 14)
      {
        counter = 0;
        point.positions.clear();
        point.velocities.clear();
        point.accelerations.clear();
      }
    }


  }

  void OrangewoodInterface::executeTrajectoryCallback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
  {

    std::string package_share_directory_ = ament_index_cpp::get_package_share_directory("mw_arm_control_minimal");
    std::string path = package_share_directory_+ "/config/traj.txt";
    moveit_msgs::msg::RobotTrajectory trajectory;
    readTrajectorymsgFromFile(trajectory,path);

    if(trajectory.joint_trajectory.points.size()==0)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"No points in trajectory");

      return;
    }

    std::vector<double> trajectory_first_state = trajectory.joint_trajectory.points[0].positions;

    //unlock the thread for state updation



    //action client goal
    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory = trajectory.joint_trajectory;
    bool goal_acceptence = false;
    bool action_complete = false;

    auto goal_response_callback = [&,this](std::shared_future<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr> future)
    {

      auto goal_handle = future.get();
      if (!goal_handle)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Goal was rejected by server");
      }
      else
      {

         goal_acceptence = true;
         RCLCPP_INFO_STREAM(node_->get_logger(),"Goal accepted by server, waiting for result");
      }
     };


    auto result_callback = [&,this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
    {

      RCLCPP_INFO_STREAM(node_->get_logger(), "[ur5_hardware_interface]Trajectoy execution complete");
      action_complete = true;
      switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(),"Execution success");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN_STREAM(node_->get_logger(),"Goal was aborted");

        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN_STREAM(node_->get_logger(), "Goal was canceled");

        break;
      default:
        RCLCPP_WARN_STREAM(node_->get_logger(),"Unknown result code");

        break;
    }


    };

    auto feedback_callback = [&,this](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
    {

      std::vector<double> error {feedback->error.positions};


    };

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = goal_response_callback;
    send_goal_options.feedback_callback = feedback_callback;
    send_goal_options.result_callback = result_callback;

    joint_traj_action_client_->async_send_goal(goal_msg, send_goal_options);
    return;

  }

  void OrangewoodInterface::stopExecutuionCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(),"Cancelling all goals");
    response->success = cancelAllGoals();
  }
  bool OrangewoodInterface::cancelAllGoals()
  {
    bool cancel_status=false;
    bool cancel_success = false;
    auto cancel_callback = [&,this](rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::CancelResponse::SharedPtr cancel_response)
    {
      cancel_status = true;

      if(cancel_response->return_code == rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::CancelResponse::ERROR_NONE)
        cancel_success = true;
      else
        cancel_success = false;

    };
    joint_traj_action_client_->async_cancel_all_goals(cancel_callback); //put a callback function to make sure it is cancelled

    return(cancel_success);

  }

}

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);

  auto node_ptr = std::make_shared<rclcpp::Node>("ur5_hardware_interface",node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_ptr);

  mw_manipulation::OrangewoodInterface orangewood_hardware_interface_obj(node_ptr);

  executor.spin();
  rclcpp::shutdown();

}
