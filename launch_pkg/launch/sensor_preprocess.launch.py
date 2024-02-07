from launch import LaunchDescription, actions
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import rclpy
from rclpy.node import Node as RclpyNode
import asyncio
import os

"""
Adjusted Function Signature: The include_launch_file_if_topics_available function's 
signature is adjusted to accept the context as its first argument, which allows the 
OpaqueFunction to pass the launch context directly.

Use of asyncio.ensure_future(): This function is used to schedule the execution of 
the coroutine on the already running asyncio event loop.
"""
# Asynchronously checks if a specific topic becomes available within a given timeout.
async def check_topic_available(node, topic_name, timeout=30):
    for _ in range(timeout):
        # Check if the topic is in the list of topics currently available in the ROS graph
        if topic_name in [name for name, _ in node.get_topic_names_and_types()]:
            return True  # Topic is available
        await asyncio.sleep(1)  # Wait for 1 second before checking again
    return False  # Timeout reached, topic not available

# Asynchronously checks if a specific topic becomes available within a given timeout.
async def include_launch_file_if_topics_available(context, package, launch_file_path, topic_names):
    if not rclpy.ok():
        rclpy.init()  # Initialize ROS client library only if not already initialized
    temp_node = RclpyNode(f"temp_node_for_inclusion")
    try:
        topics_available = all([await check_topic_available(temp_node, topic_name) for topic_name in topic_names])
        if topics_available:
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
        temp_node.destroy_node()  # Clean up the temporary node
        if rclpy.ok():
            rclpy.shutdown()  # Shutdown ROS client library if it was initialized in this function


# Function to set up the launch. This will be called by the OpaqueFunction action.
def launch_setup(context):
    # Schedule the include_launch_file_if_topics_available coroutine for each node
    # This uses asyncio.ensure_future() to schedule the coroutine in the existing event loop.
    asyncio.ensure_future(include_launch_file_if_topics_available(context, 'SensorPreprocessing', 'imu_preprocessing/launch/imu_preprocessing.launch.py', ['/imu_topic_1', '/imu_topic_2']))
    asyncio.ensure_future(include_launch_file_if_topics_available(context, 'odom_fusion_package', 'path/to/odom_fusion_launch.py', ['/imu_preprocessed']))
    asyncio.ensure_future(include_launch_file_if_topics_available(context, 'sensor_fusion_package', 'path/to/sensor_fusion_launch.py', ['/odom_preprocessed']))

# Generate the launch description. This is the entry point for the launch file.
def generate_launch_description():
    return LaunchDescription([
        # OpaqueFunction is used to execute the launch_setup function
        # which schedules our coroutines for execution.
        OpaqueFunction(function=launch_setup),
    ])
