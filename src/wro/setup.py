import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wro'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        #        ('share/ament_index/resource_index/packages'),
        ('share/' + package_name, ['package.xml']),
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
            'images = wro.wro:main',
            'stop = wro.stop:main',
            'kreis = wro.kreis:main',
        ],
    },
)

