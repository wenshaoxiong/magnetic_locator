from setuptools import setup
import os
from glob import glob

package_name = 'magnetic_locator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含所有的 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 包含所有的 URDF 文件
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # 包含所有的 World 文件
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wsx',
    maintainer_email='wsx@todo.todo',
    description='Magnetic Localization Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'magnet_mover = magnetic_locator.magnet_mover:main',
            'sensor_simulator = magnetic_locator.sensor_simulator:main',
            'localization_solver = magnetic_locator.localization_solver:main',
        ],
    },
)