import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cooperative_intercept_fake_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'scripts'),
         [path for path in glob(os.path.join('scripts', '*'))
          if os.path.isfile(path)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hw',
    maintainer_email='user@example.com',
    description=(
        'Synthetic 3-vs-1 cooperative interception virtual simulation '
        '(SE(3) control chain, cooperative metrics, visualization).'),
    license='MIT',
    entry_points={
        'console_scripts': [
            'cooperative_intercept_sim = '
            'cooperative_intercept_fake_sim.simulator:main',
            'hybrid_2v1_sim = '
            'cooperative_intercept_fake_sim.hybrid_sim:main',
            'wingman_offboard = '
            'cooperative_intercept_fake_sim.wingman_offboard:main',
        ],
    },
)
