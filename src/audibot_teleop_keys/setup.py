from setuptools import find_packages, setup

package_name = 'audibot_teleop_keys'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keys = audibot_teleop_keys.teleop_keys:main',
        ],
    },
)
