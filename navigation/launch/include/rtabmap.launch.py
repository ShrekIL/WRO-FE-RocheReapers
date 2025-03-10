from launch_ros.actions import Node
from launch import LaunchDescription
from launch import LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan': True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'queue_size': 50,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          # 'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          # 'Vis/CorType': '0',
          # 'OdomF2M/MaxSize': '4000',
          # 'Vis/MaxFeatures': '2000',
          # 'Optimizer/Slam2D': 'true',
          'grid_size': '20',
          'Grid/Sensor': 'true',
          # 'RGBD/ProximityPathMaxNeighbors': '10',
          # 'proj_max_ground_height': '0.01',
          # 'proj_max_ground_angle': '10',
    }

    remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('rgb/image', '/ascamera/camera_publisher/rgb0/image'),
            ('rgb/camera_info', '/ascamera/camera_publisher/rgb0/camera_info'),
            ('depth/image', '/ascamera/camera_publisher/depth0/image_raw'),
            ('odom', '/odom'),
            ('scan','/scan_raw'),
          ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval': 0.008, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        # Localization mode:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
