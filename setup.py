from setuptools import find_packages, setup

package_name = 'hil_crazyflie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/hil_crazyflie']),
        ('share/hil_crazyflie', ['package.xml']),

        # ⬅ install launch file
        ('share/hil_crazyflie/launch', [
            'launch/quad_launcher.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neefu',
    maintainer_email='neefu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
