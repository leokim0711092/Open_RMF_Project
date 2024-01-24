from setuptools import setup
import os
from glob import glob

package_name = 'starbots_fleet_adapters'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=starbots_fleet_adapters.fleet_adapter:main',
            'robot_api_server_barista=starbots_fleet_adapters.robot_api_server_barista:main',
            'robot_api_server_turtleE=starbots_fleet_adapters.robot_api_server_turtleE:main'

        ],
    },
)
