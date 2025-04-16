from setuptools import setup

package_name = 'joint_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/view_model.launch.py']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/joint_model.urdf']),
        ('share/' + package_name + '/urdf', ['urdf/joint_model.macro.urdf']),
        ('share/' + package_name + '/config', ['config/config.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matsjen',
    maintainer_email='matswiikjensen@gmail.no',
    description='Package for URDF visualization in RViz',
    license='TODO',
    tests_require=['pytest'],
    entry_points={},
)
