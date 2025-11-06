from setuptools import setup, find_packages
package_name = 'g03_prii3_move_jetbot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/collision_avoidance.launch.py',
            'launch/draw_number_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'draw_number = g03_prii3_move_jetbot.draw_number:main',
            'collision_avoidance = g03_prii3_move_jetbot.collision_avoidance_node:main',
            'draw_and_avoid = g03_prii3_move_jetbot.draw_and_avoid_node:main',
        ],
    },
)

