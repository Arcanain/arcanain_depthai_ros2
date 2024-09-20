from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'arcanain_depthai_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),  # detect python package
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),  
        ('share/' + package_name, ['package.xml']),  # location of install package.xml
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  
    ],
    install_requires=['setuptools'],  
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for OAK-D lite camera testing',
    license='Apache License 2.0',
    tests_require=['pytest'],  
    entry_points={
        'console_scripts': [
            'camera_detection_test = python_programs.camera_detection_test:main',
            'camera_node = python_programs.camera_node:main',
            'image_topic_test = python_programs.image_topic_test:main',
            'inference_topic_test = python_programs.inference_topic_test:main',

        ],
    },
)
