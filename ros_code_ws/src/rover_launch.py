import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    Launch file for Lunar Rover (Ubuntu side).
    Starts the web_video_server, qos_bridge, and xbee_rover_bridge simultaneously.
    """
    
    import os
    
    # Get the directory where this launch file is located (ros_code)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    qos_bridge_path = os.path.join(current_dir, 'qos_bridge.py')
    xbee_bridge_path = os.path.join(current_dir, 'xbee_rover_bridge.py')

    # Run web_video_server
    web_video_server = ExecuteProcess(
        cmd=['ros2', 'run', 'web_video_server', 'web_video_server'],
        output='screen'
    )

    # Run QoS Bridge (Converts BEST_EFFORT camera to RELIABLE)
    qos_bridge = ExecuteProcess(
        cmd=['python3', qos_bridge_path],
        output='screen'
    )

    # Run XBee Bridge (JSON telemetry sender/receiver)
    xbee_bridge = ExecuteProcess(
        cmd=['python3', xbee_bridge_path],
        output='screen'
    )

    return LaunchDescription([
        web_video_server,
        qos_bridge,
        xbee_bridge
    ])
