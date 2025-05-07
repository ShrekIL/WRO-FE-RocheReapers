from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    parameters=[{
          'queue_size':20,
          'frame_id':'depth_cam_link',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True}]

    remappings=[
          ('rgb/image', '/ascamera/camera_publisher/rgb0/image'), 
          ('rgb/camera_info', '/ascamera/camera_publisher/rgb0/camera_info'),
          ('depth/image', '/ascamera/camera_publisher/depth0/image_raw')]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Nodes to launch

        Node(
            package='rtabmap_ros', executable='rtabmapviz',
            parameters=parameters,
            remappings=remappings),

    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
