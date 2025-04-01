from setuptools import find_packages, setup

package_name = 'reference_input_node'

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
    maintainer='herman',
    maintainer_email='herman.h.gran@hotmail.com',
    description='Python server client',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = reference_input_node.client_member_function:main',
        ],
    },
)
