from setuptools import find_packages, setup

package_name = 'object_finder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sagar-venkatesh',
    maintainer_email='sagarv812@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = object_finder.controller_node:main',
        ],
    },
)
