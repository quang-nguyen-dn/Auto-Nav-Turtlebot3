from setuptools import setup

package_name = 'tb3_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quang',
    maintainer_email='quang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_driving = tb3_driving.autonomous_driving:main',
            'controller = tb3_driving.controller:main'
        ],
    },
)
