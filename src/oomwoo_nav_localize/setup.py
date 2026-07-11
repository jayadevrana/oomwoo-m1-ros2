from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'oomwoo_nav_localize'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jayadev Rana',
    maintainer_email='jayadevrana@users.noreply.github.com',
    description='Localization and kidnapped-robot recovery on a known map.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kidnap_recovery = oomwoo_nav_localize.kidnap_recovery_node:main',
        ],
    },
)
