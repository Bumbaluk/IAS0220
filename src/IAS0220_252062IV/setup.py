from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'IAS0220_252062IV'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # ✅ Install urdf files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # ✅ Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukas bumba',
    maintainer_email='lukabu@taltech.ee',
    description='Package ready for task 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_walker = IAS0220_252062IV.random_walker:main',
            'position_calculator= IAS0220_252062IV.position_calculator:main',
        ],
    },

)
