from setuptools import find_packages, setup

package_name = 'ball_tracking'

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
        "ball_tracker = ball_tracking.detect:main",
        "ball_tracker2=ball_tracking.detect2:main",
        "ball_tracker3=ball_tracking.detect3:main",
        "ball_tracker4=ball_tracking.detect4:main",
        "testnode=ball_tracking.testnode:main",
        "videocapture=ball_tracking.videocapture:main",
        "testnode2=ball_tracking.testnode2:main",
        "testnode3=ball_tracking.testnode3:main"
        ],
    },
)
