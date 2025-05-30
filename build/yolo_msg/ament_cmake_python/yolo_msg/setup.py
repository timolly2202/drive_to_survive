from setuptools import find_packages
from setuptools import setup

setup(
    name='yolo_msg',
    version='0.0.0',
    packages=find_packages(
        include=('yolo_msg', 'yolo_msg.*')),
)
