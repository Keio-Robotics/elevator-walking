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
    description="Relative movement CLI for Unitree Go2 using Unitree SDK 2.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "move_relative = elevator_walking.move_relative:main",
        ],
    },
)
