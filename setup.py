from setuptools import find_packages, setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localization_launch.py']),
        ('share/' + package_name + '/config', ['config/mapper_params_localization.yaml']),
    ],
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