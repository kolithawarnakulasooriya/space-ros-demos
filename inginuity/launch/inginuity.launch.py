from launch import LaunchDescription
from os import environ, path
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import SetParameter, Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    mars_inginuity_demos_path = get_package_share_directory('inginuity')
    mars_inginuity_models_path = get_package_share_directory('simulation')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), mars_inginuity_demos_path])}
    
    mars_world_model = path.join(mars_inginuity_demos_path, 'worlds', 'mars_inginuity.world')
    urdf_model_path = path.join(mars_inginuity_models_path, 'models', 'ingenuity_helicopter','urdf', 'helicopter.urdf.xacro')
    
    doc = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': doc.toxml()}
    
    start_world = ExecuteProcess(
        cmd=['ign gazebo', mars_world_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    propellers_node = Node(
        package="inginuity",
        executable="propellors.py"
    )

    spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'inginuity_helicopter',
            '-topic', robot_description,
            '-z', '-8.8',
            '-allow_renaming', 'true'
        ],
        output='screen' 
    )


    ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
            output='screen')

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description])

    load_propellor_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'propellor_controller'],
        output='screen'
    )

    load_body_elevation_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'elevation_controller'],
        output='screen'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    component_state_msg = '{name: "IgnitionSystem", target_state: {id: 3, label: ""}}'

    ## a hack to resolve current bug
    set_hardware_interface_active = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            'controller_manager/set_hardware_component_state',
            'controller_manager_msgs/srv/SetHardwareComponentState',
            component_state_msg]
    )

    return LaunchDescription([
        start_world,
        robot_state_publisher,
        spawn,
        propellers_node,
        ros_gz_bridge,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[set_hardware_interface_active],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_hardware_interface_active,
                on_exit=[load_propellor_velocity_controller,
                         load_body_elevation_controller],
            )
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])