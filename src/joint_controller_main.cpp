#include <rclcpp/rclcpp.hpp>
#include <ur_gz/joint_position_controller.hpp>
#include <ur_gz/joint_trajectory_controller.hpp>

int main(int argc, char* argv[])
{
    //creat ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_controller");
    // variable
    std::vector<std::string> joint_names;
    std::vector<std::string> gz_joint_topics;
    int update_rate;
    // parameters
    ros_node->declare_parameter("joint_names", {});
    ros_node->declare_parameter("gz_joint_topics", {});
    ros_node->declare_parameter("rate", 200);
    joint_names = ros_node->get_parameter("joint_names").as_string_array();
    gz_joint_topics = ros_node->get_parameter("gz_joint_topics").as_string_array();
    update_rate = ros_node->get_parameter("rate").as_int();
    // create trajectory controller
    auto joint_trajectory_controller = std::make_shared<ur_gz::JointTrajectoryController>(ros_node,
        joint_names, "set_joint_trajectory", gz_joint_topics ,update_rate);
    // create position controller 
    auto joint_position_controller = std::make_shared<ur_gz::JointPositionController>(ros_node,
        joint_names, "set_joint_state", gz_joint_topics);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
