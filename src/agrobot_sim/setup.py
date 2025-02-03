from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agrobot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds/*'))),
        (os.path.join('share', package_name, 'meshes', 'mini_greenhouse'), glob(os.path.join('meshes', 'mini_greenhouse/*'))),
        (os.path.join('share', package_name, 'models', 'mini_greenhouse'), glob(os.path.join('models', 'mini_greenhouse/*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aim',
    maintainer_email='aim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
