from setuptools import find_packages, setup

package_name = 'mscvt_ros'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='Yash Shrivastava',
    maintainer_email='shrivastavayash26@gmail.com',
    description='A multiagent system for Campus Visitor Tour',
    license='UNKNOWN',
    entry_points={
        'console_scripts': [
            'system = mscvt_ros.main:main',
        ],
    },
)
