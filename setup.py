from setuptools import find_packages, setup

package_name = 'fleet_mapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/single_scene_mapping.launch.py',
            'launch/multi_scene_mapping.launch.py',
            'launch/fleet_mapping.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nuscenes_ingestor = fleet_mapper.nuscenes_ingestor:main'
        ],
    },
)
