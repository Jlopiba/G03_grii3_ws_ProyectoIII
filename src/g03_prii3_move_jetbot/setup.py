from setuptools import setup
import os
from glob import glob

package_name = 'g03_prii3_move_jetbot'

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
    maintainer='usuario',
    maintainer_email='jlopiba1@upv.edu.es',
    description='Nodo para mover el Jetbot del grupo 3 con servicio de pausa y reinicio',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_jetbot = g03_prii3_move_jetbot.move_jetbot:main',
        ],
    },
)

