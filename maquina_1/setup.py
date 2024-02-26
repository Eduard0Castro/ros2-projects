from setuptools import find_packages, setup

package_name = 'maquina_1'

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
            'number_publisher = maquina_1.publisher:main',
            'number_counter = maquina_1.subscriber:main' 
        ],
    },
)
