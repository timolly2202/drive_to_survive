from setuptools import find_packages, setup

package_name = 'cluster_puck'

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
    maintainer='jarred',
    maintainer_email='jarred.g.deluca@student.uts.edu.au',
    description='Laser scan to global point cloud and marker visualizer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_to_global = cluster_puck.laser_to_global:main',
        ],
    },
)
