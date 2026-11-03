from setuptools import find_packages, setup

package_name = "elevator_walking"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="unitree@todo.todo",
    description="Unitree Go2 movement and lidar distance ROS 2 tools.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "move_relative = elevator_walking.move_relative:main",
            "move_forward_distance = elevator_walking.move_forward_distance:main",
            "preview_vertical_lines = elevator_walking.preview_vertical_lines:main",
            "move_left_until_line = elevator_walking.move_left_until_line:main",
            "publish_center_distance = elevator_walking.center_distance:main",
            "publish_lidar_distance = elevator_walking.lidar_front_distance:main",
            "move_until_distance = elevator_walking.move_until_distance:main",
        ],
    },
)
