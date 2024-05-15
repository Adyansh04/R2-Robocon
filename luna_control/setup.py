from setuptools import find_packages, setup

package_name = 'luna_control'

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
            "luna_wall_align = luna_control.luna_wall_align:main",
            "luna_IMU_intial_orient = luna_control.luna_IMU_intial_orient:main",
            "luna_IMU_full_orient = luna_control.luna_IMU_full_orient:main",
            "luna_align_with_full_imu_data_error = luna_control.luna_align_with_full_imu_data_error:main",
        ],
    },
)
