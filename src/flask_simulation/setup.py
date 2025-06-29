from setuptools import find_packages, setup

package_name = 'flask_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'flask',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='park',
    maintainer_email='park@todo.todo',
    description='Flask + ROS 2 integration for image streaming and web commands',
    license='MIT',
    tests_require=['pytest'],
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'ros_flask_bridge = flask_simulation.flask_app:main',
        ],
    },
)
