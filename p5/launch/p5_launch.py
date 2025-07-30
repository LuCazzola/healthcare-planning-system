# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('p5')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    deliver_patient_cmd = Node(
        package='p5',
        executable='deliver_patient_node',
        name='deliver_patient_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    deliver_patient_cmd = Node(
        package='p5',
        executable='deliver_patient_node',
        name='deliver_patient_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    deliver_supply_cmd = Node(
        package='p5',
        executable='deliver_supply_node',
        name='deliver_supply_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    drop_down_1st_box_cmd = Node(
        package='p5',
        executable='drop_down_1st_box_node',
        name='drop_down_1st_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    drop_down_2nd_box_cmd = Node(
        package='p5',
        executable='drop_down_2nd_box_node',
        name='drop_down_2nd_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    drop_down_3rd_box_cmd = Node(
        package='p5',
        executable='drop_down_3rd_box_node',
        name='drop_down_3rd_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    drop_down_patient_cmd = Node(
        package='p5',
        executable='drop_down_patient_node',
        name='drop_down_patient_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    fill_box_cmd = Node(
        package='p5',
        executable='fill_box_node',
        name='fill_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_worker_bot_alone_cmd = Node(
        package='p5',
        executable='move_worker_bot_alone_node',
        name='move_worker_bot_alone_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_helper_bot_alone_cmd = Node(
        package='p5',
        executable='move_helper_bot_alone_node',
        name='move_helper_bot_alone_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_helper_bot_patient_cmd = Node(
        package='p5',
        executable='move_helper_bot_patient_node',
        name='move_helper_bot_patient_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_worker_bot_with_1_box_cmd = Node(
        package='p5',
        executable='move_worker_bot_with_1_box_node',
        name='move_worker_bot_with_1_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_worker_bot_with_2_boxes_cmd = Node(
        package='p5',
        executable='move_worker_bot_with_2_boxes_node',
        name='move_worker_bot_with_2_boxes_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_worker_bot_with_3_boxes_cmd = Node(
        package='p5',
        executable='move_worker_bot_with_3_boxes_node',
        name='move_worker_bot_with_3_boxes_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    pick_up_1st_box_cmd = Node(
        package='p5',
        executable='pick_up_1st_box_node',
        name='pick_up_1st_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    pick_up_2nd_box_cmd = Node(
        package='p5',
        executable='pick_up_2nd_box_node',
        name='pick_up_2nd_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    pick_up_3rd_box_cmd = Node(
        package='p5',
        executable='pick_up_3rd_box_node',
        name='pick_up_3rd_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    pick_up_patient_cmd = Node(
        package='p5',
        executable='pick_up_patient_node',
        name='pick_up_patient_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(plansys2_cmd)

    ld.add_action(deliver_patient_cmd)
    ld.add_action(deliver_supply_cmd)
    ld.add_action(drop_down_1st_box_cmd)
    ld.add_action(drop_down_2nd_box_cmd)
    ld.add_action(drop_down_3rd_box_cmd)
    ld.add_action(drop_down_patient_cmd)
    ld.add_action(fill_box_cmd)
    ld.add_action(move_worker_bot_alone_cmd)
    ld.add_action(move_helper_bot_alone_cmd)
    ld.add_action(move_helper_bot_patient_cmd)
    ld.add_action(move_worker_bot_with_1_box_cmd)
    ld.add_action(move_worker_bot_with_2_boxes_cmd)
    ld.add_action(move_worker_bot_with_3_boxes_cmd)
    ld.add_action(pick_up_1st_box_cmd)
    ld.add_action(pick_up_2nd_box_cmd)
    ld.add_action(pick_up_3rd_box_cmd)
    ld.add_action(pick_up_patient_cmd)


    return ld