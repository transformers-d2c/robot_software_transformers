from setuptools import setup

package_name = 'robot_software_transformers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev19034@iiitd.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'central_node = robot_software_transformers.central_node:main',
            'camera_node =  robot_software_transformers.camera_node:main',
            'socket_node =  robot_software_transformers.socket_node:main',
        ],
    },
)
