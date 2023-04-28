// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gz_ros2_control/underactuated_gz_system.hpp"

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef GZ_HEADERS
#include <gz/msgs/imu.pb.h>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::
#else
#include <ignition/msgs/imu.pb.h>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE ignition::transport::
#define GZ_MSGS_NAMESPACE ignition::msgs::
#endif

#include <hardware_interface/hardware_info.hpp>
// #include <transmission_interface/abduction_transmission.h>

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::UnderactuatedGazeboSimSystemInterface::ControlMethod joint_control_method;

};

struct actuatorData
{
  /// \brief Actuator's names.
  std::string name;

  /// \brief Current actuator position
  double actuator_position;

  /// \brief Current actuator velocity
  double actuator_velocity;

  /// \brief Current actuator effort
  double actuator_effort;

  /// \brief Current cmd actuator position
  double actuator_position_cmd;

  /// \brief Current cmd actuator velocity
  double actuator_velocity_cmd;

  /// \brief Current cmd actuator effort
  double actuator_effort_cmd;

  /// \brief handles to the actuators from within Gazebo
  sim::Entity sim_actuator;

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::UnderactuatedGazeboSimSystemInterface::ControlMethod actuator_control_method;
};


struct MimicJoint
{
  std::size_t joint_index;
  std::size_t mimicked_joint_index;
  double multiplier = 1.0;
  std::vector<std::string> interfaces_to_mimic;
};

class ImuData
{
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  sim::Entity sim_imu_sensors_ = sim::kNullEntity;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const GZ_MSGS_NAMESPACE IMU & _msg);
};

void ImuData::OnIMU(const GZ_MSGS_NAMESPACE IMU & _msg)
{
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class gz_ros2_control::UnderactuatedGazeboSimSystemPrivate
{
public:
  UnderactuatedGazeboSimSystemPrivate() = default;

  ~UnderactuatedGazeboSimSystemPrivate() = default;
  /// \brief Degrees of freedom.
  size_t n_dof_;

  /// \brief number of underactuated transmission.
  size_t n_underactuated_trans_;

    /// \brief number of underactuated transmission.
  size_t n_joints_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the actuator's names.
  std::vector<struct actuatorData> actuators_;

  /// \brief vector with the joints underactuated.
  std::vector<int> n_joints_trans_;

  /// \brief vector with the imus .
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager * ecm;

  /// \brief controller update rate
  int * update_rate;

  /// \brief Gazebo communication node.
  GZ_TRANSPORT_NAMESPACE Node node;

  /// \brief mapping of mimicked joints to index of joint they mimic
  std::vector<MimicJoint> mimic_joints_;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;

  // Other customs variables
  std::vector<std::string> act_names_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> act_effort_;
  std::vector<double> act_velocity_;
  std::vector<double> act_position_;
  std::vector<double> act_effort_command_;
  std::vector<double> act_effort_target_;
  std::vector<double> act_position_command_;
  std::vector<double> act_velocity_command_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<std::string> transmissions_types_;
};

namespace gz_ros2_control
{

bool UnderactuatedGazeboSimSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  int & update_rate)
{
    this->dataPtr = std::make_unique<UnderactuatedGazeboSimSystemPrivate>();
    this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

    this->nh_ = model_nh;
    this->dataPtr->ecm = &_ecm;
    this->dataPtr->n_dof_ = hardware_info.transmissions.size();
    this->dataPtr->n_underactuated_trans_ = 0;
    this->dataPtr->n_joints_ = 0;
    
    for(unsigned int j=0; j < n_dof_; j++)
    {
        this->dataPtr->n_joints_ += hardware_info.transmissions[j].joints.size();
        this->dataPtr->n_joints_trans_.push_back(hardware_info.transmissions[j].joints.size());

            if (hardware_info.transmissions[j].joints_.size() > 1)
            {
                this->dataPtr->n_underactuated_trans_ += 1;
            }
    }

    this->dataPtr->update_rate = &update_rate;

    RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);
    RCLCPP_DEBUG(this->nh_->get_logger(), "n_joints_ %lu", this->dataPtr->n_joints_);

    this->dataPtr->joints_.resize(this->dataPtr->n_joints_);

    // Lambda function
    auto get_initial_value = [this](const hardware_interface::InterfaceInfo & interface_info) {
        if (!interface_info.initial_value.empty()) {
        double value = std::stod(interface_info.initial_value);
        RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", value);
        return value;
        } else {
        return 0.0;
        }
    };

    // Resizing of parameters (WIP)
    std::string hardware_interface;
    joint_types_.resize(this->dataPtr->n_joints_);
    joint_lower_limits_.resize(this->dataPtr->n_joints_);
    joint_upper_limits_.resize(this->dataPtr->n_joints_);
    joint_effort_limits_.resize(this->dataPtr->n_joints_);
    act_names_.resize(this->dataPtr->n_underactuated_trans_);
    joint_position_.resize(this->dataPtr->n_joints_);
    joint_velocity_.resize(this->dataPtr->n_joints_);
    joint_effort_.resize(this->dataPtr->n_joints_);
    joint_effort_command_.resize(this->dataPtr->n_joints_);
    joint_position_command_.resize(this->dataPtr->n_joints_);
    joint_velocity_command_.resize(this->dataPtr->n_joints_);
    act_effort_command_.resize(this->dataPtr->n_underactuated_trans_);
    act_effort_target_.resize(this->dataPtr->n_underactuated_trans_);
    act_position_command_.resize(this->dataPtr->n_underactuated_trans_);
    act_velocity_command_.resize(this->dataPtr->n_underactuated_trans_);
    act_effort_.resize(this->dataPtr->n_underactuated_trans_);
    act_position_.resize(this->dataPtr->n_underactuated_trans_);
    act_velocity_.resize(this->dataPtr->n_underactuated_trans_);
    transmissions_.resize(this->dataPtr->n_underactuated_trans_);
    transmissions_types_.resize(this->dataPtr->n_underactuated_trans_);

    constexpr double default_gain = 0.1;
    if (!this->nh_->get_parameter_or(
        "position_proportional_gain",
        this->dataPtr->position_proportional_gain_, default_gain))
    {
        RCLCPP_WARN_STREAM(
        this->nh_->get_logger(),
        "The position_proportional_gain parameter was not defined, defaulting to: " <<
            default_gain);
    }

    if (this->dataPtr->joints_ == 0) {
        RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
        return false;
    }

    // Initialize values
    unsigned int index =0;
    int i_underactuated = -1;

    for(unsigned int j=0; j < this->dataPtr->n_dof_ ; j++)
    {
        if(n_joints_trans_[j] > 1)
        {
            i_underactuated += 1;

            if(hardware_info.transmissions[j].type == "transmission_interface/AbductionTransmission")
            {
                transmissions_types_[i_underactuated] = "AbductionTransmission";
            }

            std::vector<std::string> actuator_interfaces = hardware_info.transmissions[j].actuators[0].name;

            if (actuator_interfaces.empty())
            {
                RCLCPP_WARN_STREAM(
                this->nh_->get_logger(),
                "actuator " << hardware_info.transmissions[j].actuators[0].name <<
                        " of transmission " << hardware_info.transmissions[j].name << " does not specify any hardware interface. " <<
                        "Not adding it to the robot hardware simulation.");
                continue;
            }  

            hardware_interface = actuator_interfaces.front();   

            // Add data from transmission
            this->dataPtr->act_names_[i_underactuated] = hardware_info.transmissions[j].actuators[0].name;
            this->dataPtr->act_position_[i_underactuated] = 0.0;
            this->dataPtr->act_velocity_[i_underactuated] = 0.0;
            this->dataPtr->act_effort_[i_underactuated] = 0.0;  // N/m for continuous joints
            this->dataPtr->act_effort_command_[i_underactuated] = 0.0;
            this->dataPtr->act_effort_target_[i_underactuated] = 0.0;
            this->dataPtr->act_position_command_[i_underactuated] = 0.0;
            this->dataPtr->act_velocity_command_[i_underactuated] = 0.0;

            //debug
            RCLCPP_DEBUG_STREAM(this->nh_->get_logger(),
                "Loading actuator '" << this->dataPtr->act_names_[i_underactuated] 
                << "' of type '" << hardware_interface << "'");

            RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

            double initial_position = std::numeric_limits<double>::quiet_NaN();
            double initial_velocity = std::numeric_limits<double>::quiet_NaN();
            double initial_effort = std::numeric_limits<double>::quiet_NaN();

            // register the state handles
            auto & actuator_info = hardware_info.transmissions[j].actuators[0];
            std::string actuator_name = this->dataPtr->act_names_[i_underactuated];
            std::string suffix = "";

            for (unsigned int i = 0; i < actuator_info.state_interfaces.size(); ++i) {
                if (actuator_info.state_interfaces[i].name == "position") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
                    this->dataPtr->state_interfaces_.emplace_back(
                    actuator_name + suffix,
                    hardware_interface::HW_IF_POSITION,
                    &this->dataPtr->actuators_[j].actuator_position);
                    initial_position = get_initial_value(actuator_info.state_interfaces[i]);
                    this->dataPtr->actuators_[j].actuator_position = initial_position;
                }
                if (actuator_info.state_interfaces[i].name == "velocity") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
                    this->dataPtr->state_interfaces_.emplace_back(
                    actuator_name + suffix,
                    hardware_interface::HW_IF_VELOCITY,
                    &this->dataPtr->actuators_[j].actuator_velocity);
                    initial_velocity = get_initial_value(actuator_info.state_interfaces[i]);
                    this->dataPtr->actuators_[j].actuator_velocity = initial_velocity;
                }
                if (actuator_info.state_interfaces[i].name == "effort") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
                    this->dataPtr->state_interfaces_.emplace_back(
                    actuator_name + suffix,
                    hardware_interface::HW_IF_EFFORT,
                    &this->dataPtr->actuators_[j].actuator_effort);
                    initial_effort = get_initial_value(actuator_info.state_interfaces[i]);
                    this->dataPtr->actuators_[j].actuator_effort = initial_effort;
                }
            }

            RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t Actuators Command:");

            // register the command handles
            for (unsigned int i = 0; i < actuator_info.command_interfaces.size(); ++i) {
                if (actuator_info.command_interfaces[i].name == "position") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
                    this->dataPtr->command_interfaces_.emplace_back(
                    actuator_name + suffix,
                    hardware_interface::HW_IF_POSITION,
                    &this->dataPtr->actuators_[j].actuator_position_cmd);
                    if (!std::isnan(initial_position)) {
                    this->dataPtr->actuators_[j].actuator_position_cmd = initial_position;
                    }
                } else if (actuator_info.command_interfaces[i].name == "velocity") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
                    this->dataPtr->command_interfaces_.emplace_back(
                    actuator_name + suffix,
                    hardware_interface::HW_IF_VELOCITY,
                    &this->dataPtr->actuators_[j].actuator_velocity_cmd);
                    if (!std::isnan(initial_velocity)) {
                    this->dataPtr->actuators_[j].actuator_velocity_cmd = initial_velocity;
                    }
                } else if (actuator_info.command_interfaces[i].name == "effort") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
                    this->dataPtr->command_interfaces_.emplace_back(
                    actuator_name + suffix,
                    hardware_interface::HW_IF_EFFORT,
                    &this->dataPtr->actuators_[j].actuator_effort_cmd);
                    if (!std::isnan(initial_effort)) {
                    this->dataPtr->actuators_[j].actuator_effort_cmd = initial_effort;
                    }
                }
                // independently of existence of command interface set initial value if defined
                if (!std::isnan(initial_position)) {
                    this->dataPtr->actuators_[j].actuator_position = initial_position;
                }
                if (!std::isnan(initial_velocity)) {
                    this->dataPtr->actuators_[j].actuator_velocity = initial_velocity;
                }
            }
            
        }
        else
        {
            RCLCPP_WARN_STREAM(
                this->nh_->get_logger(),
                "Not an underactuated system");
        }

    }

  for (unsigned int j = 0; j < this->dataPtr->n_joints_; j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    sim::Entity simjoint = enableJoints[joint_name];
    this->dataPtr->joints_[j].sim_joint = simjoint;

    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointPosition().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointVelocity().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointVelocity());
    }

    // Create joint force component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointForce().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointForce());
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);
    std::string suffix = "";

    // check if joint is mimicked
    if (joint_info.parameters.find("mimic") != joint_info.parameters.end()) {
      const auto mimicked_joint = joint_info.parameters.at("mimic");
      const auto mimicked_joint_it = std::find_if(
        hardware_info.joints.begin(), hardware_info.joints.end(),
        [&mimicked_joint](const hardware_interface::ComponentInfo & info) {
          return info.name == mimicked_joint;
        });
      if (mimicked_joint_it == hardware_info.joints.end()) {
        throw std::runtime_error(
                std::string("Mimicked joint '") + mimicked_joint + "' not found");
      }

      MimicJoint mimic_joint;
      mimic_joint.joint_index = j;
      mimic_joint.mimicked_joint_index = std::distance(
        hardware_info.joints.begin(), mimicked_joint_it);
      auto param_it = joint_info.parameters.find("multiplier");
      if (param_it != joint_info.parameters.end()) {
        mimic_joint.multiplier = std::stod(joint_info.parameters.at("multiplier"));
      } else {
        mimic_joint.multiplier = 1.0;
      }

      // check joint info of mimicked joint
      auto & joint_info_mimicked = hardware_info.joints[mimic_joint.mimicked_joint_index];
      const auto state_mimicked_interface = std::find_if(
        joint_info_mimicked.state_interfaces.begin(), joint_info_mimicked.state_interfaces.end(),
        [&mimic_joint](const hardware_interface::InterfaceInfo & interface_info) {
          bool pos = interface_info.name == "position";
          if (pos) {mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_POSITION);}
          bool vel = interface_info.name == "velocity";
          if (vel) {mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_VELOCITY);}
          bool eff = interface_info.name == "effort";
          if (vel) {mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_EFFORT);}
          return pos || vel || eff;
        });
      if (state_mimicked_interface == joint_info_mimicked.state_interfaces.end()) {
        throw std::runtime_error(
                std::string(
                  "For mimic joint '") + joint_info.name +
                "' no state interface was found in mimicked joint '" + mimicked_joint +
                " ' to mimic");
      }
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(),
        "Joint '" << joint_name << "'is mimicking joint '" << mimicked_joint << "' with mutiplier: "
                  << mimic_joint.multiplier);
      this->dataPtr->mimic_joints_.push_back(mimic_joint);
      suffix = "_mimic";
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t Joints Command:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      } else if (joint_info.command_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort_cmd);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
    }
  }

  registerSensors(hardware_info);

  return true;
}

void UnderactuatedGazeboSimSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // Collect gazebo sensor handles
  size_t n_sensors = hardware_info.sensors.size();
  std::vector<hardware_interface::ComponentInfo> sensor_components_;

  for (unsigned int j = 0; j < n_sensors; j++) {
    hardware_interface::ComponentInfo component = hardware_info.sensors[j];
    sensor_components_.push_back(component);
  }
  // This is split in two steps: Count the number and type of sensor and associate the interfaces
  // So we have resize only once the structures where the data will be stored, and we can safely
  // use pointers to the structures

  this->dataPtr->ecm->Each<sim::components::Imu,
    sim::components::Name>(
    [&](const sim::Entity & _entity,
    const sim::components::Imu *,
    const sim::components::Name * _name) -> bool
    {
      auto imuData = std::make_shared<ImuData>();
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(_entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }

      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      imuData->name = _name->Data();
      imuData->sim_imu_sensors_ = _entity;

      hardware_interface::ComponentInfo component;
      for (auto & comp : sensor_components_) {
        if (comp.name == _name->Data()) {
          component = comp;
        }
      }

      static const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0},
        {"orientation.y", 1},
        {"orientation.z", 2},
        {"orientation.w", 3},
        {"angular_velocity.x", 4},
        {"angular_velocity.y", 5},
        {"angular_velocity.z", 6},
        {"linear_acceleration.x", 7},
        {"linear_acceleration.y", 8},
        {"linear_acceleration.z", 9},
      };

      for (const auto & state_interface : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        this->dataPtr->state_interfaces_.emplace_back(
          imuData->name,
          state_interface.name,
          &imuData->imu_sensor_data_[data_index]);
      }
      this->dataPtr->imus_.push_back(imuData);
      return true;
    });
}

CallbackReturn
UnderactuatedGazeboSimSystem::on_init(const hardware_interface::HardwareInfo & actuator_info)
{
  RCLCPP_WARN(this->nh_->get_logger(), "On init...");
  if (hardware_interface::SystemInterface::on_init(actuator_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn UnderactuatedGazeboSimSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
UnderactuatedGazeboSimSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
UnderactuatedGazeboSimSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn UnderactuatedGazeboSimSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn UnderactuatedGazeboSimSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type UnderactuatedGazeboSimSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    // Get the joint velocity
    const auto * jointVelocity =
      this->dataPtr->ecm->Component<sim::components::JointVelocity>(
      this->dataPtr->joints_[i].sim_joint);

    // TODO(ahcorde): Revisit this part gazebosim/ign-physics#124
    // Get the joint force
    // const auto * jointForce =
    //   _ecm.Component<sim::components::JointForce>(
    //   this->dataPtr->sim_joints_[j]);

    // Get the joint position
    const auto * jointPositions =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[i].sim_joint);

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    // this->dataPtr->joint_effort_[j] = jointForce->Data()[0];
  }

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
    if (this->dataPtr->imus_[i]->topicName.empty()) {
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(this->dataPtr->imus_[i]->sim_imu_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->imus_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "IMU " << this->dataPtr->imus_[i]->name <<
            " has a topic name: " << sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(
          this->dataPtr->imus_[i]->topicName, &ImuData::OnIMU,
          this->dataPtr->imus_[i].get());
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
UnderactuatedGazeboSimSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
    
  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    for (const std::string & interface_name : stop_interfaces) {
      // Clear joint control method bits corresponding to stop interfaces
      if (interface_name == (this->dataPtr->joints_[j].name + "/" +
        hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(VELOCITY & EFFORT);
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(POSITION & EFFORT);
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(POSITION & VELOCITY);
      }

      if (interface_name == (this->dataPtr->actuators_[j].name + "/" +
        hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->actuators_[j].actuator_control_method &=
          static_cast<ControlMethod_>(VELOCITY & EFFORT);
      } else if (interface_name == (this->dataPtr->actuators_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->actuators_[j].actuator_control_method &=
          static_cast<ControlMethod_>(POSITION & EFFORT);
      } else if (interface_name == (this->dataPtr->actuators_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->actuators_[j].actuator_control_method &=
          static_cast<ControlMethod_>(POSITION & VELOCITY);
      }
    }

    // Set joint control method bits corresponding to start interfaces
    for (const std::string & interface_name : start_interfaces) {

      // For joints
      if (interface_name == (this->dataPtr->joints_[j].name + "/" +
        hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
      }

      // For actuators
      if (interface_name == (this->dataPtr->actuators_[j].name + "/" +
        hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->actuators_[j].actuator_control_method |= POSITION;
      } else if (interface_name == (this->dataPtr->actuators_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->actuators_[j].actuator_control_method |= VELOCITY;
      } else if (interface_name == (this->dataPtr->actuators_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->actuators_[j].actuator_control_method |= EFFORT;
      }

    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnderactuatedGazeboSimSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
    // JOINTS
    for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) 
    {
        if (this->dataPtr->joints_[i].joint_control_method & VELOCITY) {
          if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint))
          {
              this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              sim::components::JointVelocityCmd({0}));
          } else {
              const auto jointVelCmd =
              this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);
              *jointVelCmd = sim::components::JointVelocityCmd(
              {this->dataPtr->joints_[i].joint_velocity_cmd});
          }
        } else if (this->dataPtr->joints_[i].joint_control_method & POSITION) 
        {
          // Get error in position
          double error;
          error = (this->dataPtr->joints_[i].joint_position -
              this->dataPtr->joints_[i].joint_position_cmd) * *this->dataPtr->update_rate;

          // Calculate target velcity
          double target_vel = -this->dataPtr->position_proportional_gain_ * error;

          auto vel =
              this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);

          if (vel == nullptr) {
              this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              sim::components::JointVelocityCmd({target_vel}));
          } else if (!vel->Data().empty()) {
              vel->Data()[0] = target_vel;
          }
        } else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) 
        {
          if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
              this->dataPtr->joints_[i].sim_joint))
          {
              this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              sim::components::JointForceCmd({0}));
          } else {
              const auto jointEffortCmd =
              this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
              this->dataPtr->joints_[i].sim_joint);
              *jointEffortCmd = sim::components::JointForceCmd(
              {this->dataPtr->joints_[i].joint_effort_cmd});
          }
        } else 
        {
          // Fallback case is a velocity command of zero
          double target_vel = 0.0;
          auto vel =
              this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);

          if (vel == nullptr) {
              this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              sim::components::JointVelocityCmd({target_vel}));
          } else if (!vel->Data().empty()) {
              vel->Data()[0] = target_vel;
          } else if (!vel->Data().empty()) {
              vel->Data()[0] = target_vel;
          }
        }
    }

    // ACTUATORS
    for (unsigned int i = 0; i < this->dataPtr->actuators_.size(); ++i) 
    {
        if (this->dataPtr->actuators_[i].actuator_control_method & VELOCITY) 
        {
          if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint))
          {
              this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              sim::components::JointVelocityCmd({0}));
          } else {
              const auto jointVelCmd =
              this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);
              *jointVelCmd = sim::components::JointVelocityCmd(
              {this->dataPtr->joints_[i].joint_velocity_cmd});
          }
        } else if (this->dataPtr->joints_[i].joint_control_method & POSITION) 
        {
          // Get error in position
          double error;
          error = (this->dataPtr->joints_[i].joint_position -
              this->dataPtr->joints_[i].joint_position_cmd) * *this->dataPtr->update_rate;

          // Calculate target velcity
          double target_vel = -this->dataPtr->position_proportional_gain_ * error;

          auto vel =
              this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);

          if (vel == nullptr) {
              this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              sim::components::JointVelocityCmd({target_vel}));
          } else if (!vel->Data().empty()) {
              vel->Data()[0] = target_vel;
          }
        } else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) 
        {
          if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
              this->dataPtr->joints_[i].sim_joint))
          {
              this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              sim::components::JointForceCmd({0}));
          } else 
          {
              const auto jointEffortCmd =
              this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
              this->dataPtr->joints_[i].sim_joint);
              *jointEffortCmd = sim::components::JointForceCmd(
              {this->dataPtr->joints_[i].joint_effort_cmd});
          }
        } else 
        {
          // Fallback case is a velocity command of zero
          double target_vel = 0.0;
          auto vel =
              this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);

          if (vel == nullptr) {
              this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              sim::components::JointVelocityCmd({target_vel}));
          } else if (!vel->Data().empty()) {
              vel->Data()[0] = target_vel;
          } else if (!vel->Data().empty()) {
              vel->Data()[0] = target_vel;
          }
        }
    }

  // set values of all mimic joints with respect to mimicked joint
  for (const auto & mimic_joint : this->dataPtr->mimic_joints_) {
    for (const auto & mimic_interface : mimic_joint.interfaces_to_mimic) {
      if (mimic_interface == "position") {
        // Get the joint position
        double position_mimicked_joint =
          this->dataPtr->ecm->Component<sim::components::JointPosition>(
          this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)->Data()[0];

        double position_mimic_joint =
          this->dataPtr->ecm->Component<sim::components::JointPosition>(
          this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)->Data()[0];

        double position_error =
          position_mimic_joint - position_mimicked_joint * mimic_joint.multiplier;

        double velocity_sp = (-1.0) * position_error * (*this->dataPtr->update_rate);

        auto vel =
          this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
          this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

        if (vel == nullptr) {
          this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
            sim::components::JointVelocityCmd({velocity_sp}));
        } else if (!vel->Data().empty()) {
          vel->Data()[0] = velocity_sp;
        }
      }
      if (mimic_interface == "velocity") {
        // get the velocity of mimicked joint
        double velocity_mimicked_joint =
          this->dataPtr->ecm->Component<sim::components::JointVelocity>(
          this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)->Data()[0];

        if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint))
        {
          this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
            sim::components::JointVelocityCmd({0}));
        } else {
          const auto jointVelCmd =
            this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);
          *jointVelCmd = sim::components::JointVelocityCmd(
            {mimic_joint.multiplier * velocity_mimicked_joint});
        }
      }
      if (mimic_interface == "effort") {
        // TODO(ahcorde): Revisit this part ignitionrobotics/ign-physics#124
        // Get the joint force
        // const auto * jointForce =
        //   _ecm.Component<sim::components::JointForce>(
        //   this->dataPtr->sim_joints_[j]);
        if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint))
        {
          this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
            sim::components::JointForceCmd({0}));
        } else {
          const auto jointEffortCmd =
            this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);
          *jointEffortCmd = sim::components::JointForceCmd(
            {mimic_joint.multiplier *
              this->dataPtr->joints_[mimic_joint.mimicked_joint_index].joint_effort});
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace gz_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gz_ros2_control::UnderactuatedGazeboSimSystem, gz_ros2_control::UnderactuatedGazeboSimSystemInterface)
