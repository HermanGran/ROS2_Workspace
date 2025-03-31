from setuptools import find_packages, setup

package_name = 'tutorial_py_srvcli'

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
    description='Python server client tutorial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = tutorial_py_srvcli.service_member_function:main',
            'client = tutorial_py_srvcli.client_member_function:main',
        ],
    },
)
