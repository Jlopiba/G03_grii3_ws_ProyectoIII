from setuptools import setup
import os
from glob import glob

package_name = 'g03_prii3_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorge',
    maintainer_email='jorge@example.com',
    description='Paquete g03_prii3_turtlesim con nodo y launch',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_turtle = g03_prii3_turtlesim.auto_turtle:main',
        ],
    },
)

