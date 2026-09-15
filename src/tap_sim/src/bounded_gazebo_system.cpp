// Provisional actuator bounds executed in Gazebo, not a calibrated motor model.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <gz/sim/Joint.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "tap_sim/actuator_bounds.hpp"
#include "tap_sim/actuator_gate.hpp"
#include "tap_sim/leg_velocity_reference.hpp"

namespace tap_sim
{
class BoundedGazeboSystem : public gz_ros2_control::GazeboSimSystemInterface
{
  struct JointData
  {
    std::string name;
    gz::sim::Entity entity;
    bool wheel = false;
    bool active = false;
    double position = 0, velocity = 0, effort = 0, command = 0;
  };
  std::vector<JointData> joints_;
  gz::sim::EntityComponentManager * ecm_ = nullptr;
  double leg_speed_limit_ = 12.566370614359172;
  double leg_velocity_damping_ = 0.0;
  ActuatorGate gate_;
  LegVelocityReference reference_;
  std::mutex gate_mutex_;
  static double wallSeconds()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_sub_;

public:
  bool initSim(rclcpp::Node::SharedPtr & node, std::map<std::string, sim::Entity> & entities,
    const hardware_interface::HardwareInfo & info, sim::EntityComponentManager & ecm,
    unsigned int) override
  {
    ecm_ = &ecm;
    if (info.hardware_parameters.count("leg_speed_limit")) {
      leg_speed_limit_ = std::stod(info.hardware_parameters.at("leg_speed_limit"));
    }
    if (!std::isfinite(leg_speed_limit_) || leg_speed_limit_ <= 0 ||
      leg_speed_limit_ > 28.274333882308138) {return false;}
    if (info.hardware_parameters.count("leg_velocity_damping")) {
      leg_velocity_damping_ = std::stod(info.hardware_parameters.at("leg_velocity_damping"));
    }
    if (!std::isfinite(leg_velocity_damping_) || leg_velocity_damping_ < 0) {return false;}
    estop_sub_ = node->create_subscription<std_msgs::msg::Bool>("/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(gate_mutex_);
        gate_.stop(msg->data);
      });
    enable_sub_ = node->create_subscription<std_msgs::msg::Bool>("/actuator_enable", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(gate_mutex_);
        gate_.receive(msg->data, wallSeconds());
      });
    reference_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/leg_velocity_reference", 1, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(gate_mutex_);
        const double stamp = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
        reference_.receive(msg->name, msg->velocity, stamp, wallSeconds(), leg_speed_limit_);
      });
    joints_.reserve(info.joints.size());
    for (const auto & item : info.joints) {
      if (!entities.count(item.name)) {return false;}
      JointData data;
      data.name = item.name;
      data.entity = entities.at(item.name);
      data.wheel = item.name == "cad_left_wheel" || item.name == "cad_right_wheel";
      if (item.command_interfaces.size() != 1 ||
        item.command_interfaces[0].name != (data.wheel ? "velocity" : "effort")) {return false;}
      for (const auto & state : item.state_interfaces) {
        if (state.name == "position" && !state.initial_value.empty()) {
          data.position = std::stod(state.initial_value);
        }
      }
      gz::sim::Joint joint(data.entity);
      joint.EnablePositionCheck(ecm);
      joint.EnableVelocityCheck(ecm);
      joint.ResetPosition(ecm, {data.position});
      joints_.push_back(data);
    }
    return joints_.size() == 6;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> result;
    for (auto & j : joints_) {
      result.emplace_back(j.name, "position", &j.position);
      result.emplace_back(j.name, "velocity", &j.velocity);
      // Effort explicitly means applied actuator torque, excluding contact/passive damping.
      result.emplace_back(j.name, "effort", &j.effort);
    }
    return result;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> result;
    for (auto & j : joints_) {
      result.emplace_back(j.name, j.wheel ? "velocity" : "effort", &j.command);
    }
    return result;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    for (auto & j : joints_) {
      const auto key = j.name + (j.wheel ? "/velocity" : "/effort");
      if (std::find(stop.begin(), stop.end(), key) != stop.end()) {j.active = false; j.command = 0;}
      if (std::find(start.begin(), start.end(), key) != start.end()) {j.active = true;}
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (auto & j : joints_) {
      gz::sim::Joint joint(j.entity);
      const auto p = joint.Position(*ecm_), v = joint.Velocity(*ecm_);
      if (p && !p->empty()) {j.position = (*p)[0];}
      if (v && !v->empty()) {j.velocity = (*v)[0];}
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    bool enabled;
    std::array<double, 4> references{};
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      const double wall = wallSeconds();
      enabled = reference_.allowTorque(gate_, time.seconds(), wall, references);
    }
    for (auto & j : joints_) {
      gz::sim::Joint joint(j.entity);
      const auto speed = joint.Velocity(*ecm_);
      const double velocity = speed && !speed->empty() ? (*speed)[0] : 0;
      double torque = 0;
      if (j.active && enabled && std::isfinite(j.command) && std::isfinite(velocity)) {
        // No extra positive mechanical work beyond nominal speed; external overspeed
        // is not instantaneously clipped, and braking remains torque-bounded.
        double target_velocity = 0;
        if (!j.wheel) {
          const auto it = std::find(LegVelocityReference::names.begin(),
            LegVelocityReference::names.end(), j.name);
          if (it != LegVelocityReference::names.end()) {
            target_velocity = references[it - LegVelocityReference::names.begin()];
          }
        }
        torque = boundedTorque(j.wheel, j.command, velocity, leg_speed_limit_,
          leg_velocity_damping_, target_velocity);
      }
      j.effort = torque;
      joint.SetForce(*ecm_, {torque});
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    for (auto & j : joints_) {
      j.active = false; j.command = 0; j.effort = 0;
      gz::sim::Joint(j.entity).SetForce(*ecm_, {0});
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
};
}  // namespace tap_sim
PLUGINLIB_EXPORT_CLASS(tap_sim::BoundedGazeboSystem, gz_ros2_control::GazeboSimSystemInterface)
