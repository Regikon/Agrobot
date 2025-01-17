from setuptools import setup
# from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

settings = generate_distutils_setup(
    packages=[
    ],
)

setup(install_requires=["numpy", "rospkg", "empy"], **settings)
