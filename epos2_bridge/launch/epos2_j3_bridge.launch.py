from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='epos2_bridge',
            executable='epos2_j3_bridge',
            name='epos2_j3_bridge',
            output='screen',
            parameters=[
                {'can_interface': 'can0'},
                {'joint_name': 'joint_3'},
                {'encoder_qc_per_motor_rev': 4096.0},
                {'gear_ratio_motor_per_joint_rev': 100.0},
                {'sign': 1.0},
                {'zero_offset_qc': 0.0},
                {'joint_state_rate_hz': 50.0},
                {'telemetry_rate_hz': 10.0},
                {'ipm_default_segment_ms': 10},
                {'fault_clear_on_startup': False},
                {'enable_on_startup': False},
                {'force_ipm_on_startup': True},
            ],
        )
    ])
