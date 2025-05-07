import os
from glob import glob
from setuptools import setup

package_name = 'multi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'launch/include'), glob(os.path.join('launch/include', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'formation_update = multi.formation_update:main',
            'slave_tf_listener = multi.slave_tf_listener:main',
            'tf_listen = multi.tf_listen:main',
            'tf_publish = multi.tf_publish:main',
            'costmap_publish = multi.costmap_publish:main',            
        ],
    },
)
