from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'bot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goblue',
    maintainer_email='goblue@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rotate_wheels=bot_description.wheel_control:main"
        ],
    },
)
