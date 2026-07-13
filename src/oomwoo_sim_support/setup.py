from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'oomwoo_sim_support'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # gz model overrides (loaded ahead of the stock models via
        # GZ_SIM_RESOURCE_PATH precedence in sim_bringup). TableMarble carries a
        # convex-decomposition collision so its mesh has real physics headless.
        (os.path.join('share', package_name, 'models', 'TableMarble'),
            ['models/TableMarble/model.sdf', 'models/TableMarble/model.config']),
        (os.path.join('share', package_name, 'models', 'TableMarble', 'meshes'),
            glob('models/TableMarble/meshes/*')),
        (os.path.join('share', package_name, 'models', 'TableMarble', 'textures'),
            glob('models/TableMarble/textures/*')),
        (os.path.join('share', package_name, 'models', 'TableMarble',
                      'materials', 'textures'),
            glob('models/TableMarble/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jayadev Rana',
    maintainer_email='jayadevrana@users.noreply.github.com',
    description='Headless sim bringup, ground-truth measurement, and regression tests.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth = oomwoo_sim_support.ground_truth_node:main',
            'coverage_meter = oomwoo_sim_support.coverage_meter_node:main',
            'kidnap_injector = oomwoo_sim_support.kidnap_injector_node:main',
            'initialpose_pub = oomwoo_sim_support.initialpose_pub_node:main',
            'reloc_regression_runner = oomwoo_sim_support.reloc_regression_runner:main',
            'coverage_regression_runner = oomwoo_sim_support.coverage_regression_runner:main',
        ],
    },
)
