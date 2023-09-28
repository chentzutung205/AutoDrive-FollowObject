from setuptools import find_packages, setup

package_name = 'bb8_object_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tzu-Tung Chen',
    maintainer_email='tchen604@gatech.edu',
    description='Let BB8 find and follow the object',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'find_object=bb8_object_follower.find_object:main',
        	'debug_node=bb8_object_follower.debug_node:main',
        	'rotate_robot=bb8_object_follower.rotate_robot:main'
        ],
    },
)
