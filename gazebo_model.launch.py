import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    
    robotXacroName='differential_drive_robot'
    namePackage = 'driverless_robot'
    
    nameWorld = 'model/empty_world.world'
    
    pkg_path = os.path.join(get_package_share_directory('driverless_robot'))
    xacro_file = os.path.join(pkg_path,'model','robot.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(namePackage), 'launch', 'model.launch.py'
        )]), launch_arguments={'use_sim_time':'true'}.items())
    
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]), launch_arguments={'world': nameWorld}.items()
    )
    
    
    
    #gazeboLaunch = IncludeLaunchDescription(gazebo, launch_arguments={'world':nameWorld}.items())
    
    spawnModelNode=Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', robotXacroName], output='screen')
    
    
    return LaunchDescription(
        [rsp,
        gazebo, spawnModelNode]
    )