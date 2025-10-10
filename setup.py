from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dynamic_manipulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sastra',
    maintainer_email='sastra@example.com',
    description='Dynamic manipulator package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qwen_node = scripts.qwen_inference:main'
        ],
    },
)

