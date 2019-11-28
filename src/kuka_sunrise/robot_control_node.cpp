/*
 * robot_control_node.cpp
 *
 *  Created on: Nov 11, 2019
 *      Author: rosdeveloper
 */

#include "kuka_sunrise/robot_control_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace kuka_sunrise
{

RobotControlNode::RobotControlNode() :
    LifecycleNode("robot_control"), close_requested_(false)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto command_srv_callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                     std_srvs::srv::SetBool::Request::SharedPtr request,
                                     std_srvs::srv::SetBool::Response::SharedPtr response)
                                     {
                                       (void)request_header;
                                       if(request->data == true)
                                       {
                                         response->success = this->activate();
                                       }
                                       else
                                       {
                                         response->success = this->deactivate();
                                       }
                                     };
  set_command_state_service_ = this->create_service<std_srvs::srv::SetBool>("robot_control/set_commanding_state",
                                                                            command_srv_callback);
}

RobotControlNode::~RobotControlNode()
{
  printf("in destructor");
}

void RobotControlNode::runClientApplication()
{
  client_application_->connect(30200, NULL);
  bool success = true;
  while (success && !close_requested_.load())
  {
    success = client_application_->step();

    if (client_->robotState().getSessionState() == KUKA::FRI::IDLE)
    {
      break;
    }
  }
  if (!success)
  {
    //TODO handle
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::on_configure(
    const rclcpp_lifecycle::State &state)
{
  (void)state;
  //TODO change stack size with setrlimit rlimit_stack?
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
  {
    RCLCPP_ERROR(get_logger(), "mlockall error");
    RCLCPP_ERROR(get_logger(), std::strerror(errno));
    return ERROR;
  }

  struct sched_param param;
  param.sched_priority = 90;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
  {
    RCLCPP_ERROR(get_logger(), "setscheduler error");
    RCLCPP_ERROR(get_logger(), std::strerror(errno));
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::on_cleanup(
    const rclcpp_lifecycle::State &state)
{
  (void)state;

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::on_shutdown(
    const rclcpp_lifecycle::State &state)
{
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result = SUCCESS;
  switch (state.id())
  {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      if (this->on_deactivate(get_current_state()) != SUCCESS)
      {
        result = ERROR;
        break;
      }
      this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;
    default:
      break;
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::on_activate(
    const rclcpp_lifecycle::State &state)
{
  (void)state;
  client_ = std::make_unique<RobotControlClient>(this->shared_from_this());
  client_application_ = std::make_unique<KUKA::FRI::ClientApplication>(udp_connection_, *client_);
  client_application_thread_ = std::make_unique<pthread_t>();

  auto run_app = [](void *robot_control_node) -> void*
  {
    static_cast<RobotControlNode*>(robot_control_node)->runClientApplication();
    return nullptr;
  };

  if (pthread_create(client_application_thread_.get(), nullptr, run_app, this))
  {
    RCLCPP_ERROR(get_logger(), "pthread_create error");
    RCLCPP_ERROR(get_logger(), std::strerror(errno));
    return ERROR;
  }

  return SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::on_deactivate(
    const rclcpp_lifecycle::State &state)
{
  (void)state;
  if(this->isActive()){
    this->deactivate();
  }
  close_requested_.store(true);
  pthread_join(*client_application_thread_, NULL); //TODO can hang here, apply timeout
  close_requested_.store(false);
  client_application_->disconnect();
  client_.reset();
  client_application_.reset();
  client_application_thread_.reset();
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::on_error(
    const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "An error occured");
  return SUCCESS;
}

bool RobotControlNode::activate()
{
  this->ActivatableInterface::activate();
  return client_->activateControl();
}

bool RobotControlNode::deactivate()
{
  this->ActivatableInterface::deactivate();
  return client_->deactivateControl();
}

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_sunrise::RobotControlNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
