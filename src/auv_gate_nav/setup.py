from setuptools import find_packages, setup

package_name = 'auv_gate_nav'

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
    maintainer='hanakamar',
    maintainer_email='hanakamar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_gate = auv_gate_nav.navigate_gate:main',
            'test_auv_motion = auv_gate_nav.test_auv_motion:main',
        ],
    },
)
