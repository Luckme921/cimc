from glob import glob
from setuptools import find_packages, setup

package_name = 'cimc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml', 'README.md',
            '任务协调与手眼ABB桥接节点说明.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luckme',
    maintainer_email='luckme@todo.todo',
    description='ABB data receiver and eccentric welding-gun motor control nodes.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'motor_control_node = cimc.motor_control_node:main',
            'data_receiver_node = cimc.data_receiver_node:main',
            'weld_task_coordinator_node = cimc.weld_task_coordinator_node:main',
            'handeye_abb_bridge_node = cimc.handeye_abb_bridge_node:main',
        ],
    },
)
