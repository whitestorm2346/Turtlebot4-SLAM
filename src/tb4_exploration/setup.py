from setuptools import find_packages, setup

package_name = 'tb4_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tsehsun',
    maintainer_email='tsehsun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'exploration_manager = tb4_exploration.exploration_manager:main',
            'map_visualizer = tb4_exploration.map_visualizer:main',
        ],
    },
)
