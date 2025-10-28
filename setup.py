from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: [
            '../control_config.json',
            '../maps/*.yaml',
            '../maps/*.pgm',
        ],
    },
    install_requires=['setuptools', 'click', 'prompt_toolkit'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='DOME robot control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = control.main:main',
        ],
    },
)