from setuptools import find_packages, setup

package_name = 'ROS2_OAKDlite_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nkn4ryu',
    maintainer_email='nkn4ryu@todo.todo',
    description='A simple ROS2 node for controlling DepthAI camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = ROS2_OAKDlite_test.camera_node.camera_node:main',
            'camera_detect_test = ROS2_OAKDlite_test.camera_detection_test:main', # check wether camera can be detected by ROS2 system
            'image_topic_test = ROS2_OAKDlite_test.image_topic_test:main', # confirn that camera node can publish image raw topic
            'inference_topic_test = ROS2_OAKDlite_test.inference_topic_test:main' # put in NNnode pipeline and publish inference topic
        ],
    },
)
