from setuptools import find_packages, setup

package_name = 'r2_py'

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
    maintainer='adyansh04',
    maintainer_email='gupta.adyansh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "number_publisher = r2_py.number_publisher:main",
            "number_counter = r2_py.number_counter:main",
            "motor_server_fake = r2_py.motor_server_fake:main",
            "motor_client = r2_py.motor_client:main",
            "test_node1 = r2_py.test_node1:main",
            "test_node2 = r2_py.test_node2:main",
            "force_stop = r2_py.force_stop:main",
            "quat_to_rpy = r2_py.quaternion_to_rpy:main",
            "ps4 = r2_py.ps4:main",
            "cmd_vel_slow_pub = r2_py.cmd_vel_slow_pub:main",
        ],
    },
)
