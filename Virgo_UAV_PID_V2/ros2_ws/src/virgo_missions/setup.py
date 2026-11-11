from setuptools import find_packages, setup

package_name = 'virgo_missions'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duc',
    maintainer_email='duc@local',
    description='ROS 2 Humble Mission Suite for Virgo OMAV (H-Frame Omnidirectional UAV)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_node = virgo_missions.virgo_mission_node:main',
            'tilt_node = virgo_missions.virgo_tilt_node:main',
            'se3_node = virgo_missions.virgo_se3_geometric_node:main',
        ],
    },
)
