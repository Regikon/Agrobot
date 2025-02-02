from setuptools import find_packages, setup
import os
from glob import glob

package_name = "agrobot_urdf"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
    (os.path.join("share", package_name, "meshes", "robot_body"), glob("meshes/robot_body/*")),
    (os.path.join("share", package_name, "meshes", "mecanum_wheel"), glob("meshes/mecanum_wheel/*")),
    (os.path.join("share", package_name, "meshes", "rail_wheel"), glob("meshes/rail_wheel/*")),
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aim",
    maintainer_email="aim@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
