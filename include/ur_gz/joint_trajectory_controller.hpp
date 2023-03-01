#ifndef UR_GZ_JOINT_TRAJECTORY_CONTROLLER_H
#define UR_GZ_JOINT_TRAJECTORY_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <unordered_map>
#include <mutex>

namespace ur_gz {

class JointTrajectoryController {
public:
    JointTrajectoryController(const rclcpp::Node::SharedPtr& nh,
        const std::vector<std::string>& joint_names,
        const std::string& ros_cmd_topic,
        const std::vector<std::string>& gz_cmd_topics,
        const unsigned int update_rate);
    ~JointTrajectoryController() {};

private:
    void setJointTrajectoryCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr);
    void updatePositionTimerCb();

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<gz::transport::Node> gz_node_;
    // ros pub and sub
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr ros_cmd_joint_trajectory_sub_;
    //gazebo pub
    std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
    rclcpp::TimerBase::SharedPtr update_position_timer_;
    // joint names and map
    std::vector<std::string> joint_names_;
    size_t joint_num_;
    std::vector<double> target_positions_;
    std::unordered_map<std::string, int> joint_names_map_;
    std::mutex trajectory_mut_;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points_;
    rclcpp::Time trajectory_start_time_;
    unsigned int trajectory_index_;
    bool has_trajectory_ { false };

};
}
#endif //UR_GZ_JOINT_TRAJECTORY_CONTROLLER_H