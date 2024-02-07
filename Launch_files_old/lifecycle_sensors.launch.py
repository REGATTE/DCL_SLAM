import launch
from launch import LaunchDescription, actions
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import rclpy
from rclpy.node import Node as RclpyNode
import asyncio
import os

async def check_topic_available(node, topic_name, timeout=30):
    """
    Asynchronously check if a specific topic becomes available within a given timeout.
    """
    for _ in range(timeout):
        if topic_name in [name for name, _ in node.get_topic_names_and_types()]:
            return True
        await asyncio.sleep(1)  # Wait for 1 second before checking again
    return False

async def include_launch_file_if_topics_available(package, launch_file_path, topic_names, context):
    """
    Include a launch file if the specified topics are available.
    """
    rclpy.init()
    temp_node = RclpyNode(f"temp_node_for_inclusion")
    try:
        # Check if all provided topics are available
        topics_available = all([await check_topic_available(temp_node, topic_name) for topic_name in topic_names])
        if topics_available:
            # Construct the full path to the launch file
            full_launch_file_path = os.path.join(
                FindPackageShare(package).find(package),
                launch_file_path
            )
            context.add_completion_action(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(full_launch_file_path),
                    launch_arguments={'output': 'screen'}.items()
                )
            )
        else:
            temp_node.get_logger().error(f"Timeout waiting for topics {', '.join(topic_names)}. Launch file {launch_file_path} not included.")
    finally:
        temp_node.destroy_node()
        rclpy.shutdown()

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=lambda context: asyncio.run(include_launch_file_if_topics_available(
            'SensorPreprocessing', 'imu_preprocessing/launch/imu_preprocessing.launch.py', ['/IMU/raw_0', '/IMU/raw_1'], context))),
        OpaqueFunction(function=lambda context: asyncio.run(include_launch_file_if_topics_available(
            'SensorPreprocessing', 'odom_ekf/launch/odom_ekf.launch.py', ['/IMU/processed'], context))),
    ])
