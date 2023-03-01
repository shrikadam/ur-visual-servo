#ifndef UR_GZ_JOINT_STATE_PUBLISHER_H
#define UR_GZ_JOINT_STATE_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <map>
#include <mutex>

namespace ur_gz {

class JointStatePublisher {
public:
    JointStatePublisher(const rclcpp::Node::SharedPtr& nh,
        const std::vector<std::string>& joint_names,
        const std::string& ros_topic, 
        const std::string& gz_topic,
        const unsigned int update_rate);
    ~JointStatePublisher() {};

private:
    void jointStateTimerCb();
    //callback for Ignition
    void gzJointStateCb(const gz::msgs::Model& msg);

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<gz::transport::Node> gz_node_;
    // ros pub and sub
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ros_joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    // joint names and map
    std::vector<std::string> joint_names_;
    std::map<std::string,int> joint_names_map_;
    //joint state info recieved form gazebo
    gz::msgs::Model current_gz_joint_msg_;
    sensor_msgs::msg::JointState current_joint_msg_;
    std::mutex current_joint_msg_mut_;
};

}

#endif //UNIVERSAL_ROBOT_IGN_JOINT_STATE_PUBLISHER_H