from setuptools import find_packages, setup

package_name = "ros_serial_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros2",
    maintainer_email="jdedu.kr@gmail.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_pub = ros_serial_py.ros_serial_py_pub:main",
            "serial_sub = ros_serial_py.ros_serial_py_sub:main",
        ],
    },
)
