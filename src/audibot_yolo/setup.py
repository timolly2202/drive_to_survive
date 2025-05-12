from setuptools import find_packages, setup
from glob import glob

package_name = 'audibot_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim',
    maintainer_email='Timothy.J.Ollerton@student.uts.edu.au',
    description='Takes image data from audibot cameras and draws bounding boxes for image detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audibot_yolo = audibot_yolo.yolo_publisher:main',
            'front_camera_node = audibot_yolo.front_camera_node:main',
            'back_camera_node = audibot_yolo.back_camera_node:main',
            'left_camera_node = audibot_yolo.left_camera_node:main',
            'right_camera_node = audibot_yolo.right_camera_node:main',
        ],
    },
)
