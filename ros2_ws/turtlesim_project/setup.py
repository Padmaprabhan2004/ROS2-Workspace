from setuptools import find_packages, setup

package_name = 'turtlesim_project'

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
    maintainer='softroboticslabiith',
    maintainer_email='softroboticslabiith@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'spawner = turtlesim_project.turtle_spawner:main',
            'controller=turtlesim_project.turtle_controller:main'
        ],
    },
)
