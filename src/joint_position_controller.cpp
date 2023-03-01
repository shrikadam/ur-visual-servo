#include "ur_gz/joint_position_controller.hpp"

using namespace std;
using namespace ur_gz;

JointPositionController::JointPositionController(const rclcpp::Node::SharedPtr& nh,
    const std::vector<std::string>& joint_names,
    const std::string& ros_cmd_topic,
    const std::vector<std::string>& gz_cmd_topics)
{
    // ROS and Gazebo node
    nh_ = nh;
    gz_node_ = std::make_shared<gz::transport::Node>();
    //check
    if (joint_names.size() != gz_cmd_topics.size()) {
        std::cout << "[JointPositionController ERROR]:the size of arrays are not matched!" << std::endl;
        return;
    }
    joint_names_ = joint_names;
    //init joint names map
    for (size_t i = 0; i < joint_names_.size(); i++) {
        joint_names_map_[joint_names_[i]] = i;
    }
    //create ros pub and sub
    ros_cmd_joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(ros_cmd_topic, 10,
        std::bind(&JointPositionController::setJointPositionCb, this, std::placeholders::_1));
    //create ignition pub
    for (size_t i = 0; i < gz_cmd_topics.size(); i++) {
        auto pub = std::make_shared<gz::transport::Node::Publisher>(
            gz_node_->Advertise<gz::msgs::Double>(gz_cmd_topics[i]));
        gz_cmd_joint_pubs_.push_back(pub);
    }
}

void JointPositionController::setJointPositionCb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (auto i = 0u; i < msg->position.size(); ++i) {
        if (joint_names_map_.find(msg->name[i]) != joint_names_map_.end()) {
            //find joint name in `joint_names_` .
            int idx = joint_names_map_[msg->name[i]];
            gz::msgs::Double gz_msg;
            gz_msg.set_data(msg->position[i]);
            gz_cmd_joint_pubs_[idx]->Publish(gz_msg);
        }
    }
}
