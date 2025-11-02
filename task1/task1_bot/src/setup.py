from setuptools import find_packages, setup

package_name = 'task1_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/statepublisher.py']),
        ('share/' + package_name + '/launch', ['launch/gazebolaunch.py']),
        ('share/' + package_name + '/meshes', ['meshes/base_link.STL']),
        ('share/' + package_name + '/meshes', ['meshes/steeering3.STL']),
        ('share/' + package_name + '/meshes', ['meshes/steering1.STL']),
        ('share/' + package_name + '/meshes', ['meshes/steering2.STL']),
        ('share/' + package_name + '/meshes', ['meshes/steering4.STL']),
        ('share/' + package_name + '/meshes', ['meshes/wheel1.STL']),
        ('share/' + package_name + '/meshes', ['meshes/wheel2.STL']),
        ('share/' + package_name + '/meshes', ['meshes/wheel3.STL']),
        ('share/' + package_name + '/meshes', ['meshes/wheel4.STL']),
        ('share/' + package_name + '/urdf', ['urdf/rover_description.urdf']),
        ('share/' + package_name + '/world', ['world/model.world']),
        ('share/' + package_name + '/world', ['world/model.config']),
        ('share/' + package_name + '/config', ['config/diff_drive.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranabpandey31',
    maintainer_email='pranabpandey06@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cmd_velNode=task1_bot.forward_kinematics:main"
        ],
    },
)
