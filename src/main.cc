#include <chrono>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "leonard_interfaces/srv/go_to_point.hpp"

#include "adapter-pkg/dobot-manipulator-model-node.hh"

using DobotNode =
    palette_server_api::lib::ros_foxy::adapter_pkg::DobotManipulatorModelNode;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

template <class T> auto DecorateTimeExecute(T &&func)
{
  auto wrapper = [func](auto &&...args)
  {
    auto begin = std::chrono::steady_clock::now();
    auto result = func(args...);
    auto end = std::chrono::steady_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    RCLCPP_WARN(rclcpp::get_logger("UTILS"), "Time execute: %d",
                duration.count());
    return result;
  };
  return wrapper;
}

/**
 * @brief Класс для управления манипулятором.
 * Добавляет на сцену плоскость. Подписывается на топик, по которому
 * получает целевые точки.
 *
 */
class ManipulatorController
    : public palette_server_api::lib::ros_foxy::adapter_pkg::MiniNode
{
public:
  ManipulatorController(rclcpp::Executor::SharedPtr executor)
      : MiniNode(executor, "manipulator_cntl"),
        mManipulator(executor, "dobot",
                     palette_server_api::dto::props::Manipulator{1.0f})
  {
    node_->declare_parameter("go_to_point_service", "test_service");
    mManipulator.set_eef_speed(1.0f);
    mManipulator.GoTo("home");
    // mSubscription = node_->create_subscription<geometry_msgs::msg::Point>(
    //     node_->get_parameter("go_to_point_service").as_string(), 10,
    //     std::bind(&ManipulatorController::Callback, this,
    //               std::placeholders::_1));
    mGoToPointService =
        node_->create_service<leonard_interfaces::srv::GoToPoint>(
            node_->get_parameter("go_to_point_service").as_string(),
            [this](
                std::shared_ptr<
                    leonard_interfaces::srv::GoToPoint::Request> const request,
                std::shared_ptr<leonard_interfaces::srv::GoToPoint::Response>
                    response)
            {
              auto begin = std::chrono::steady_clock::now();
              GoToPoint(request, response);
              auto end = std::chrono::steady_clock::now();
              auto duration =
                  std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                        begin);
              RCLCPP_WARN(rclcpp::get_logger("UTILS"), "Time execute: %d",
                          duration.count());
            });
    // Add collisionObject
    MoveGroup moveGroup(node_, "dobot_arm");
    moveit_msgs::msg::CollisionObject collisionObject;
    collisionObject.header.frame_id = moveGroup.getPlanningFrame();
    collisionObject.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.05;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 1;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose boxPose;
    boxPose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    boxPose.position.x = -0.5;
    boxPose.position.y = 0;
    boxPose.position.z = 0.75;

    collisionObject.primitives.push_back(primitive);
    collisionObject.primitive_poses.push_back(boxPose);
    collisionObject.operation = collisionObject.ADD;
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
    planningSceneInterface.applyCollisionObject(collisionObject);
  }
  ~ManipulatorController() = default;

private:
  DobotNode mManipulator;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr mSubscription;
  rclcpp::Service<leonard_interfaces::srv::GoToPoint>::SharedPtr
      mGoToPointService;

  void Callback(geometry_msgs::msg::Point::SharedPtr const msg)
  {
    palette_server_api::dto::Pose pose;
    pose.position.x = -0.45;
    pose.position.y = 0.75f - msg->x;
    pose.position.z = 1.0f - msg->y;
    pose.orientation.x = -0.5f;
    pose.orientation.y = -0.5f;
    pose.orientation.z = 0.5f;
    pose.orientation.w = 0.5f;
    try
    {
      mManipulator.GoTo(pose);
    }
    catch (std::runtime_error const &e)
    {
      RCLCPP_ERROR(node_->get_logger(), e.what());
    }
  }

  void GoToPoint(
      std::shared_ptr<leonard_interfaces::srv::GoToPoint::Request> const
          request,
      std::shared_ptr<leonard_interfaces::srv::GoToPoint::Response> response)
  {
    palette_server_api::dto::Pose pose;
    pose.position.x = -0.45;
    pose.position.y = 0.75f - request->target.x;
    pose.position.z = 1.0f - request->target.y;
    pose.orientation.x = -0.5f;
    pose.orientation.y = -0.5f;
    pose.orientation.z = 0.5f;
    pose.orientation.w = 0.5f;
    try
    {
      mManipulator.GoTo(pose);
      response->success = true;
    }
    catch (std::runtime_error const &e)
    {
      RCLCPP_ERROR(node_->get_logger(), e.what());
      response->success = false;
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto executor = rclcpp::executors::SingleThreadedExecutor::make_shared();
  auto ctrl = std::make_unique<ManipulatorController>(executor);
  executor->spin();
  rclcpp::shutdown();
}
