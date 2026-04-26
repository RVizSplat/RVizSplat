import glob
import os

from setuptools import find_packages, setup

package_name = 'gsplat_publisher'

splats_dir = os.path.join(os.path.dirname(__file__), '..', 'splats')
ply_files = glob.glob(os.path.join(splats_dir, '*.ply'))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'splats'), ply_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Publishes latched SplatArray messages loaded from a PLY file.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ply_splat_publisher = gsplat_publisher.ply_splat_publisher:main',
        ],
    },
)
