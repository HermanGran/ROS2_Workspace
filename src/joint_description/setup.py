from setuptools import setup

package_name = 'joint_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/view_model.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matswj',
    maintainer_email='matswiikjensen@gmail.no',
    description='Package for URDF visualization in RViz',
    license='TODO',
    tests_require=['pytest'],
    entry_points={},
)
