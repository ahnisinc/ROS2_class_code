from setuptools import find_packages, setup

package_name = 'ros_cv_center'

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
    maintainer='ros2',
    maintainer_email='jdedu.kr@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_center_pub = ros_cv_center.cam_center_pub:main',
            'cam_center_sub = ros_cv_center.cam_center_sub:main',
        ],
    },
)
