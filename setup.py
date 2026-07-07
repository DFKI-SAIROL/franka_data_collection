from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'franka_data_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Data collection package for Franka Research 3 robots',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'data_collector_main = franka_data_collection.data_collector_node:main',
        ],
    },
)
