#include "ur_gz/joint_state_publisher.hpp"

using namespace std;
using namespace ur_gz;

JointStatePublisher::JointStatePublisher(const rclcpp::Node::SharedPtr& nh,
        const std::vector<std::string>& joint_names,
        const std::string& ros_topic, 
        const std::string& gz_topic,
        const unsigned int update_rate)
{
    // ROS and Gazebo node
    nh_ = nh;
    gz_node_ = std::make_shared<gz::transport::Node>();

    joint_names_ = joint_names;
    for (size_t i = 0; i < joint_names_.size(); i++) {
        joint_names_map_[joint_names_[i]]=i;
    }
    //create ros pub and sub
    ros_joint_state_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>(ros_topic, 10);
    auto period = std::chrono::microseconds(1000000 / update_rate);
    joint_state_timer_ = nh_->create_wall_timer(period, std::bind(&JointStatePublisher::jointStateTimerCb, this));
    //create gazebo sub
    gz_node_->Subscribe(gz_topic, &JointStatePublisher::gzJointStateCb, this);
    //init current_joint_msg_
    for (auto i = 0u; i < joint_names_.size(); ++i) {
        current_joint_msg_.name.push_back(joint_names_[i]);
        current_joint_msg_.position.push_back(0);
        current_joint_msg_.velocity.push_back(0);
        current_joint_msg_.effort.push_back(0);
    }
}

void JointStatePublisher::jointStateTimerCb()
{
    gz::msgs::Model model_msg;
    {
        std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
        model_msg=current_gz_joint_msg_;
    }
    
    //create  JointState msg
    current_joint_msg_.header.stamp = rclcpp::Clock().now();
    current_joint_msg_.header.frame_id = model_msg.name();
    for(int i = 0; i < model_msg.joint_size () ; ++i){
        if (joint_names_map_.find(model_msg.joint(i).name()) != joint_names_map_.end()) {
            int idx=joint_names_map_[model_msg.joint(i).name()];
            current_joint_msg_.position[idx]=model_msg.joint(i).axis1().position();
            current_joint_msg_.velocity[idx]=model_msg.joint(i).axis1().velocity();
            current_joint_msg_.effort[idx]=model_msg.joint(i).axis1().force();
        }
    }
    ros_joint_state_pub_->publish(current_joint_msg_);
}

void JointStatePublisher::gzJointStateCb(const gz::msgs::Model& msg)
{
    std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
    current_gz_joint_msg_ = msg;
}