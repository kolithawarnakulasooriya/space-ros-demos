from launch import LaunchDescription
from os import environ, path
from launch.actions import ExecuteProcess
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mars_inginuty_demos_path = get_package_share_directory('inginuty')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), mars_inginuty_demos_path])}
    
    mars_world_model = path.join(mars_inginuty_demos_path, 'worlds', 'mars_inginuty.world')
    
    start_world = ExecuteProcess(
        cmd=['ign gazebo', mars_world_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        start_world,
    ])