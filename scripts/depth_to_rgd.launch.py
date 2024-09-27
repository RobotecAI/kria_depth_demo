from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):


    rgbd_pc = ComposableNodeContainer(
            name='container0',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('rgb/camera_info', '/left/camera_info'),
                                ('rgb/image_rect_color', '/left'),
                                ('depth_registered/image_rect','/camera_image_depth'),
                                ('/points','/pointcloud')]
                ),
            ],
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    
    rgbd_pc1 = ComposableNodeContainer(
        name='container0',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[('rgb/camera_info', '/left/camera_info'),
                            ('rgb/image_rect_color', '/left'),
                            ('depth_registered/image_rect','/disparity_map'),
                            ('/points','/pointcloudFGPA')]
            ),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    

    nodes_to_start = [
        rgbd_pc,
        rgbd_pc1
    ]

    return nodes_to_start
