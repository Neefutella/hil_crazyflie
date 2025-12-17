from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/quadcopter/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
            '/X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators',
        ],
        output='screen'
    )

    controller_path = '/home/neefu/ws/ros2/src/hil_crazyflie/hil_crazyflie/ctrl.py'
    pose_to_z_path = '/home/neefu/ws/ros2/src/hil_crazyflie/hil_crazyflie/pose_to_z_node.py'
    pose_stabilizer_mixer_path = '/home/neefu/ws/ros2/src/hil_crazyflie/hil_crazyflie/mixer.py'
    thrust_to_actuators_path = '/home/neefu/ws/ros2/src/hil_crazyflie/hil_crazyflie/thrust_actuator.py'

    pose_to_z = ExecuteProcess(cmd=['python3', pose_to_z_path], output='screen')
    controller = ExecuteProcess(cmd=['python3', controller_path], output='screen')
    pose_stabilizer_mixer = ExecuteProcess(cmd=['python3', pose_stabilizer_mixer_path], output='screen')
    thrust_to_actuators = ExecuteProcess(cmd=['python3', thrust_to_actuators_path], output='screen')

    return LaunchDescription([
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
        bridge,
        pose_to_z,
        controller,
        pose_stabilizer_mixer,
        thrust_to_actuators,
    ])
