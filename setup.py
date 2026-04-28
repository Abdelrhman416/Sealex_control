from setuptools import find_packages, setup

package_name = 'sealex_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/sealex_control/config',  ['config/ekf.yaml']),
        ('share/sealex_control/launch',  ['launch/gnc_with_ekf.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdelrahman',
    maintainer_email='abdelrahman@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'usv_gnc_node = sealex_control.usv_gnc_node:main',
            'imu_cal_check = sealex_control.imu_cal_check:main',
            'thruster_driver_node = sealex_control.thruster_driver_node:main',
        ],
    },
)
