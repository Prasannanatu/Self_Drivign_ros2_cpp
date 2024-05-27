# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():

#   package_dir=get_package_share_directory('self_driving_cpp')
#   world_file = os.path.join(package_dir,'worlds','self_driving_car.world')

#   return LaunchDescription([

#         ExecuteProcess(
#             cmd=['gazebo', '--verbose',world_file, '-s', 'libgazebo_ros_factory.so'],
#             output='screen'),


#         Node(
#                 package='self_driving_cpp',
#                 executable='lights_spawner.bash',
#                 name='Lights_installer',
#                 output='screen'),




#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

  package_dir = get_package_share_directory('self_driving_cpp')
  world_file = os.path.join(package_dir, 'worlds', 'Lane_follow_test.world')
  lights_spawner_script = os.path.join(package_dir, 'lib', 'self_driving_cpp', 'lights_spawner.bash')

  return LaunchDescription([
      ExecuteProcess(
          cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
          output='screen'
      ),
    #   ExecuteProcess(
    #       cmd=[lights_spawner_script],
    #       output='screen'
    #   ),
      Node(
        package='self_driving_cpp',
        executable='lights_spawner.bash',
        name='Lights_installer',
        output='screen'),
      Node(
          package='self_driving_cpp',
          executable='video_recorder',
          name='video_recorder',
          output='screen'
      ),
      Node(
          package='self_driving_cpp',
          executable='computer_vision_node',
          name='computer_vision_node',
          output='screen'
      ),
  ])
