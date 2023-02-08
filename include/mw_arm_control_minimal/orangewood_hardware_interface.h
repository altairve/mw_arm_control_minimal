/*
* File Name: ur5_hardware_interface.h
* Author: Safar V
* Email: safar@mowito.in
* Brief: Header File for interfacing with ur5 controller
*/
#ifndef UR5_HARDWARE_INTERFACE_H
#define UR5_HARDWARE_INTERFACE_H
#include "rclcpp/rclcpp.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "moveit_msgs/msg/robot_trajectory.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "rclcpp/rclcpp.hpp"
#include<fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <functional>
#include <chrono>
//mutex lock stuff
#include <thread>
#include <mutex>
#include <condition_variable>
#include <shared_mutex>


namespace mw_manipulation
{
  /**
   * @brief Class object that takes care of all hardware interfacing and associated tasks
   */
  class OrangewoodInterface
  {
    public:
      /**
        * @brief Constructor
        * @param node node pointer that host all the servers , this node should be added to a
        * [MultiThreadedExecutor](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1executors_1_1MultiThreadedExecutor.html) and spun.
        */
      OrangewoodInterface(rclcpp::Node::SharedPtr node_ptr);

      /**
      * @brief Callback function to the service that execute a given trajectory
      * @param request shared_ptr of std_srvs::srv::Trigger srv
      * @param response shared_ptr of std_srvs::srv::Trigger srv
      */
      void executeTrajectoryCallback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
      /**
      * @brief Callback function to the service that stops execution of a trajectory
      * @param request shared_ptr of std_msgs::srv::SetBool srv
      * @param response shared_ptr of std_msgs::srv::SetBool srv
      */
      void stopExecutuionCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,std_srvs::srv::SetBool::Response::SharedPtr response);
      /**
      * @brief Funciton that cancels all action goals
      * @return true if the cancellation action is succesful, else returns false
      */
      bool cancelAllGoals();
      /**
      * @brief Funciton that loads trajectory msg from txt file
      * @param trajectory empty trajectory to which we will load from a txt file
      * @param path absolute path for the trajectory file
      */
      void readTrajectorymsgFromFile(moveit_msgs::msg::RobotTrajectory &trajectory,std::string path);
      /**
      *@brief improved sting to float
      *@param fs string in number with decimal point
      *returns float same as the string given
      */
      float betterStof(std::string &fs);



    private:
      rclcpp::Node::SharedPtr node_;

      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_trajectory_service_;
      rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_execution_service_;

      rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joint_traj_action_client_;
      rclcpp::CallbackGroup::SharedPtr service_callback_group_;
      rclcpp::CallbackGroup::SharedPtr inner_callback_group_;
      


  };
}
#endif
