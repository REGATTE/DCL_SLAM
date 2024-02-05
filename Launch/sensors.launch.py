from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    # Imu Preprocessing Sensor Fusion package
    imu_preproc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 'SensorPreprocessing/imu_preprocessing/launch/imu_preprocessing.launch.py'])
    )
    # Odom Preprocessing Sensor Fusion package   
    odom_preproc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 'SensorPreprocessing/odom_ekf/launch/odom_ekf.launch.py'])
    )

    return LaunchDescription([
        imu_preproc_launch,
        odom_preproc_launch
    ])