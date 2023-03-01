#include <rclcpp/rclcpp.hpp>
#include <ur_gz/joint_state_publisher.hpp>

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_state_publisher");
    // variable
    std::vector<std::string> joint_names;
    std::string gz_topic;
    int update_rate;
    // get parameter
    ros_node->declare_parameter("joint_names", {});
    ros_node->declare_parameter("gz_topic", {});
    ros_node->declare_parameter("rate", 30);
    joint_names = ros_node->get_parameter("joint_names").as_string_array();
    gz_topic = ros_node->get_parameter("gz_topic").as_string();
    update_rate = ros_node->get_parameter("rate").as_int();

    // create publisher
    auto joint_publisher = std::make_shared<ur_gz::JointStatePublisher>(ros_node,
        joint_names, "joint_states", gz_topic,update_rate);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
