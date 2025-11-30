from setuptools import find_packages, setup

package_name = 'Coxa_control'

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
    maintainer='dd',
    maintainer_email='dd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'joint_controller = Coxa_control.joint_controller:main',
        'calibration      = Coxa_control.calibration:main'
        ],
    },
)
