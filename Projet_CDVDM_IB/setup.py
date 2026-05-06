from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'Projet_CDVDM_IB'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        # To make your rviz config file installed in your workspace
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtle',
    maintainer_email='turtle@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linefollow = Projet_CDVDM_IB.line_following_node:main',
            'corridornavigation = Projet_CDVDM_IB.corridor_navigation_node:main',
            'obstacleavoidance = Projet_CDVDM_IB.obstacle_avoidance_node:main',
            'hsvtune = Projet_CDVDM_IB.hsv_calibration_node:main',
            'handteleop=Projet_CDVDM_IB.handteleop_node:main',
        ],
    },
)
