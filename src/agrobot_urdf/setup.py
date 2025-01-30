from setuptools import find_packages, setup
import os
from glob import glob

package_name = "agrobot_urdf"


def copy_dir():
    dir_path = "meshes"
    base_dir = os.path.join(package_name, dir_path)
    for dirpath, dirnames, files in os.walk(base_dir):
        for f in files:
            yield os.path.join(dirpath.split("/", 1)[1], f)


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
]
for f in copy_dir():
    data_files.append(f)

print(data_files)

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
