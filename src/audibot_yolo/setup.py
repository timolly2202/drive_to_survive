from setuptools import find_packages, setup

package_name = 'audibot_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
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
        ],
    },
)
