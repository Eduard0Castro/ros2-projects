from setuptools import find_packages, setup

package_name = 'toggle_led_python'

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
            "server_led = toggle_led_python.server_led_panel:main",
            "battery_state = toggle_led_python.battery_state:main"
        ],
    },
)
