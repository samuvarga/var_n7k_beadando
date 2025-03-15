from setuptools import setup
import os

package_name = 'var_n7k_beadando'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/archery_target_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='he',
    maintainer_email='samuvarga24@gmail.com',
    description='TODO: Package description',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'archery_target_node = var_n7k_beadando.archery_target_node:main'
        ],
    },
)
