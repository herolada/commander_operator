from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'commander_operator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*.yaml')),
        (os.path.join('share', package_name, 'data', 'routes'), glob('data/routes/*.yaml')),
        (os.path.join('share', package_name, 'data', 'routes_wgs'), glob('data/routes_wgs/*.yaml')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Herold',
    maintainer_email='herolada@fel.cvut.cz',
    description='An interface between human operator and the (crl_)commander package.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'commander_operator_node = commander_operator.commander_operator_node:main'
        ],
    },
)
