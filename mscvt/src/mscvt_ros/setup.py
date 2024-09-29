from setuptools import find_packages, setup

package_name = 'mscvt_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name, ['rviz_config.rviz']),
    ],
    install_requires=['setuptools', 'geometry_msgs',
                      'visualization_msgs'],
    zip_safe=True,
    maintainer='yash26',
    maintainer_email='shrivastavayash26@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'campus_map_publisher = mscvt_ros.campus_map_publisher:main',
        ],
    },
)
