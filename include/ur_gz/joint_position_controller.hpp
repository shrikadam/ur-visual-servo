#ifndef UR_GZ_JOINT_POSITION_CONTROLLER_H
#define UR_GZ_JOINT_POSITION_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <unordered_map>
#include <mutex>

namespace ur_gz {

class JointPositionController {
public:
    JointPositionController(const rclcpp::Node::SharedPtr& nh,
        const std::vector<std::string>& joint_names,
        const std::string& ros_cmd_topic,
        const std::vector<std::string>& gz_cmd_topics);
    ~JointPositionController() {};

private:
    void setJointPositionCb(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<gz::transport::Node> gz_node_;
    // ros pub and sub
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ros_cmd_joint_state_sub_;
    //gazebo pub
    std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
    // joint names and map
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, int> joint_names_map_;
};
}
#endif //UR_GZ_JOINT_POSITION_CONTROLLER_H