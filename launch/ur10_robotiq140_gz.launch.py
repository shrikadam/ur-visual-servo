'''Launch ur10 ignition_simulator with ros joint position controller and state publisher'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xmacro.xmacro4sdf import XMLMacro4sdf

def generate_launch_description():
    ld = LaunchDescription()
    pkg_ros_gz = get_package_share_directory('ros_gz')
    pkg_ur_gz = get_package_share_directory('ur_gz')
    #data
    world_sdf_path=os.path.join(pkg_ur_gz, 'resource', 'worlds', 'test_world.sdf') 
    robot_xmacro_path=os.path.join(pkg_ur_gz, 'resource', 'xmacro', 'ur10_robotiq140.sdf.xmacro') 
    gz_config_path=os.path.join(pkg_ur_gz, 'gz', 'gui.config')
    # ignition_simulator launch
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': world_sdf_path + ' -r -v 2 --gui-config ' + gz_config_path,
        }.items()
    )
    ld.add_action(gazebo_sim)
    # Spawn robot
    robot_macro = XMLMacro4sdf()
    robot_macro.set_xml_file(robot_xmacro_path)
    robot_macro.generate()
    robot_xml = robot_macro.to_string()
    spawn_robot = Node(package='ros_gz', executable='create',
        arguments=['-name', 'ur10' ,'-z', '1.4', '-string', robot_xml],
        output='screen')
    ld.add_action(spawn_robot)
    # parameter for ur10 controller
    joint_names_list=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                    "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
    gz_joint_topics_list=[]
    for joint_name in joint_names_list:
        gz_joint_topics_list.append("/model/ur10/joint/%s/0/cmd_pos"%joint_name)
    
    # ros<-ign, joint state publisher for ur10
    joint_state_publisher=Node(package='ur_gz', 
                executable='joint_state_publisher',
                name="ur10_joint_state_publisher",
                parameters=[{"joint_names": joint_names_list},
                            {"gz_topic": "/world/default/model/ur10/joint_state"}
                        ],
                output='screen')
    ld.add_action(joint_state_publisher)
    #  ros->ign,  joint controller for ur10
    joint_controller=Node(package='ur_gz', 
                executable='joint_controller',
                name="ur10_joint_controller",
                parameters=[{"joint_names": joint_names_list},
                            {"gz_joint_topics": gz_joint_topics_list},
                            {"rate":200},
                           ],
                output='screen')
    ld.add_action(joint_controller)
    # ros->ign, ign bridge to control Robotiq140 Gripper
    ros_gz_bridge = Node(package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=["/model/ur10/gripper@std_msgs/msg/Bool]ignition.msgs.Boolean"],
            remappings=[
                ("/model/ur10/gripper","/gripper"),
            ],
            output='screen'
    )
    ld.add_action(ros_gz_bridge)
    return ld
