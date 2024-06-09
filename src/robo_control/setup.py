from setuptools import find_packages, setup

package_name = 'robo_control'

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
    maintainer='mac_vaibubuntu',
    maintainer_email='mac_vaibubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'controller = robo_control.controller:main',
              'aruco_detector = aruco_navigation.aruco_detector:main',
              
        ],
    },
)
