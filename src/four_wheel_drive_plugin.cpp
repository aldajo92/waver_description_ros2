#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// to print in console
#include <iostream>

namespace gazebo_plugins
{
    class FourWheelDrivePlugin : public gazebo::ModelPlugin
    {
    public:
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
        {
            // Initialize ROS node
            gazebo_ros::Node::SharedPtr ros_node = gazebo_ros::Node::Get(sdf);

            // Store model pointer
            this->model = model;

            // Subscriber to command velocities
            cmd_vel_subscriber = ros_node->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", rclcpp::QoS(10),
                std::bind(&FourWheelDrivePlugin::OnCmdVel, this, std::placeholders::_1));

            // Get joints
            left_front_joint = model->GetJoint("wheel_front_left_joint");
            right_front_joint = model->GetJoint("wheel_front_right_joint");
            left_rear_joint = model->GetJoint("wheel_back_left_joint");
            right_rear_joint = model->GetJoint("wheel_back_right_joint");

            // Check joints
            if (!left_front_joint || !right_front_joint || !left_rear_joint || !right_rear_joint)
            {
                RCLCPP_ERROR(ros_node->get_logger(), "Some joints were not found!");
                return;
            }
        }

    private:
        void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            // Simple differential drive kinematics for 4 wheel drive
            double left_speed = msg->linear.x - msg->angular.z;
            double right_speed = msg->linear.x + msg->angular.z;

            left_front_joint->SetVelocity(0, left_speed);
            right_front_joint->SetVelocity(0, right_speed);
            left_rear_joint->SetVelocity(0, left_speed);
            right_rear_joint->SetVelocity(0, right_speed);

            std::cout << "left_speed: " << left_speed << " right_speed: " << right_speed << std::endl;
        }

        gazebo::physics::ModelPtr model;
        gazebo_ros::Node::SharedPtr ros_node;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
        gazebo::physics::JointPtr left_front_joint, right_front_joint, left_rear_joint, right_rear_joint;

        double wheel_separation = 0.5; // Example value, adjust to your model
    };

    GZ_REGISTER_MODEL_PLUGIN(FourWheelDrivePlugin)
}
