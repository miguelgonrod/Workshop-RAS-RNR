from setuptools import find_packages, setup

package_name = 'taller_rnr'

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
    maintainer='miguel',
    maintainer_email='miguelgonrod2004@gmail.com',
    description='Paquete del taller RNR para aprendizaje de ROS2 con TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_publisher_angular = taller_rnr.turtlebot3_simpĺe_publisher_angular:main',
            'turtlebot3_publisher_linear = taller_rnr.turtlebot3_simpĺe_publisher_linear:main',
            'turtlebot3_subscriber = taller_rnr.turtlebot3_simple_suscriber:main',
        ],  
    },
)
