from setuptools import find_packages, setup

package_name = 'silo_tracking'

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
    maintainer='athrva',
    maintainer_email='athrvakulkarni11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "silo_tracking=silo_tracking.silo_detect:main",
            "silo_tracking2=silo_tracking.silo_tracking2:main",
        ],
    },
)
