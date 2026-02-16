#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <memory>

namespace gazebo
{
  class GimbalKinematicPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      // Store the model pointer
      this->model = _model;

      // Initialize ROS node
      this->ros_node = gazebo_ros::Node::Get(_sdf);

      // Subscribe to trajectory commands
      this->trajectory_sub = this->ros_node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/set_joint_trajectory",
        10,
        std::bind(&GimbalKinematicPlugin::OnTrajectory, this, std::placeholders::_1)
      );

      // Get joint pointers
      RCLCPP_INFO(this->ros_node->get_logger(), "Plugin attached to model: %s", this->model->GetName().c_str());
      
      // Try to find the gimbal model
      physics::ModelPtr gimbal_model = this->model->NestedModel("gimbal_3axis_vtol");
      if (gimbal_model) {
        RCLCPP_INFO(this->ros_node->get_logger(), "Found nested gimbal model: %s", gimbal_model->GetName().c_str());
        this->yaw_joint = gimbal_model->GetJoint("gimbal_yaw_joint");
        this->pitch_joint = gimbal_model->GetJoint("gimbal_pitch_joint");
        this->roll_joint = gimbal_model->GetJoint("gimbal_roll_joint");
      } else {
        RCLCPP_WARN(this->ros_node->get_logger(), "Nested gimbal model not found, searching current model joints...");
        // Fallback: search current model
        auto joints = this->model->GetJoints();
        for (const auto& joint : joints) {
            std::string name = joint->GetName();
            if (name.find("gimbal_yaw") != std::string::npos) this->yaw_joint = joint;
            if (name.find("gimbal_pitch") != std::string::npos) this->pitch_joint = joint;
            if (name.find("gimbal_roll") != std::string::npos) this->roll_joint = joint;
        }
      }

      if (this->yaw_joint) RCLCPP_INFO(this->ros_node->get_logger(), "Found yaw joint: %s", this->yaw_joint->GetName().c_str());
      if (this->pitch_joint) RCLCPP_INFO(this->ros_node->get_logger(), "Found pitch joint: %s", this->pitch_joint->GetName().c_str());
      if (this->roll_joint) RCLCPP_INFO(this->ros_node->get_logger(), "Found roll joint: %s", this->roll_joint->GetName().c_str());

      if (!this->yaw_joint) RCLCPP_ERROR(this->ros_node->get_logger(), "Failed to find gimbal_yaw_joint");
      if (!this->pitch_joint) RCLCPP_ERROR(this->ros_node->get_logger(), "Failed to find gimbal_pitch_joint");
      if (!this->roll_joint) RCLCPP_ERROR(this->ros_node->get_logger(), "Failed to find gimbal_roll_joint");

      RCLCPP_INFO(this->ros_node->get_logger(), "Gimbal kinematic control plugin loaded in namespace: %s", this->ros_node->get_namespace());
      RCLCPP_INFO(this->ros_node->get_logger(), "Subscribing to: /set_joint_trajectory");

      // Connect to the update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GimbalKinematicPlugin::OnUpdate, this));
          
      // Initialize targets (neutral by default for debugging)
      this->target_yaw = 0.0;
      this->target_pitch = 0.0; 
      this->target_roll = 0.0;
    }

  private:
    void OnUpdate()
    {
      static int counter = 0;
      if (counter++ % 1000 == 0) {
          double cur_yaw = this->yaw_joint ? this->yaw_joint->Position(0) : 0.0;
          double cur_pitch = this->pitch_joint ? this->pitch_joint->Position(0) : 0.0;
          double cur_roll = this->roll_joint ? this->roll_joint->Position(0) : 0.0;

          RCLCPP_INFO(this->ros_node->get_logger(), "OnUpdate: Targets(%.2f, %.2f, %.2f) Current(%.2f, %.2f, %.2f)", 
              this->target_yaw, this->target_pitch, this->target_roll,
              cur_yaw, cur_pitch, cur_roll);
      }

      if (this->yaw_joint) this->yaw_joint->SetPosition(0, this->target_yaw);
      if (this->pitch_joint) this->pitch_joint->SetPosition(0, this->target_pitch);
      if (this->roll_joint) this->roll_joint->SetPosition(0, this->target_roll);
      
      // Zero velocity to prevent physics drift
      if (this->yaw_joint) this->yaw_joint->SetVelocity(0, 0.0);
      if (this->pitch_joint) this->pitch_joint->SetVelocity(0, 0.0);
      if (this->roll_joint) this->roll_joint->SetVelocity(0, 0.0);
    }

    void OnTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
      if (msg->points.empty()) return;

      const auto& point = msg->points[0];
      
      RCLCPP_INFO(this->ros_node->get_logger(), "Received trajectory command: %f, %f, %f", 
                  point.positions[0], point.positions[1], point.positions[2]);

      bool updated = false;
      // Update targets
      for (size_t i = 0; i < msg->joint_names.size() && i < point.positions.size(); ++i)
      {
        const std::string& joint_name = msg->joint_names[i];
        double position = point.positions[i];

        if (joint_name.find("yaw") != std::string::npos) { this->target_yaw = position; updated = true; }
        else if (joint_name.find("pitch") != std::string::npos) { this->target_pitch = position; updated = true; }
        else if (joint_name.find("roll") != std::string::npos) { this->target_roll = position; updated = true; }
      }
      
      if (updated) {
          RCLCPP_INFO(this->ros_node->get_logger(), "Target updated: Y=%.2f, P=%.2f, R=%.2f", 
                      this->target_yaw, this->target_pitch, this->target_roll);
      } else {
          RCLCPP_WARN(this->ros_node->get_logger(), "No matching joints found in trajectory command!");
          for (const auto& name : msg->joint_names) {
              RCLCPP_WARN(this->ros_node->get_logger(), " - Msg joint: %s", name.c_str());
          }
      }
    }

    physics::ModelPtr model;
    physics::JointPtr yaw_joint;
    physics::JointPtr pitch_joint;
    physics::JointPtr roll_joint;
    gazebo_ros::Node::SharedPtr ros_node;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub;
    event::ConnectionPtr updateConnection;
    
    double target_yaw;
    double target_pitch;
    double target_roll;
  };

  GZ_REGISTER_MODEL_PLUGIN(GimbalKinematicPlugin)
}
