from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'mapping_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('script_navigatable_v3/launch/*.py')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('script_navigatable_v3/config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@robot.com',
    description='Mapping Manager',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
