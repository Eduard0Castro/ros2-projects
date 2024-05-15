from setuptools import find_packages, setup

package_name = 'python_pkg'

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
    maintainer='eduardocastro',
    maintainer_email='americogomes1@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "python_node = python_pkg.python_test:main", 
            "robot_station = python_pkg.robot_station:main",
            "smartphone = python_pkg.smartphone:main", 
            "add_two_ints_server = python_pkg.add_two_ints:main",
            "client_add_two_ints = python_pkg.client_add_two_ints:main", 
            "class_client_add_two_ints = python_pkg.class_client_add_two_ints:main", 
            "robot_hardware_sensor = python_pkg.robot_my_custom_msg:main",
            "subs_robot = python_pkg.subs_robot_custom:main",
            "resistance_server = python_pkg.resistance_server:main",
            "resistance_client = python_pkg.resistance_client:main"
        ],
    },
)
