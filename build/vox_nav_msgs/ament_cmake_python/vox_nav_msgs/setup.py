from setuptools import find_packages
from setuptools import setup

setup(
    name='vox_nav_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('vox_nav_msgs', 'vox_nav_msgs.*')),
)
