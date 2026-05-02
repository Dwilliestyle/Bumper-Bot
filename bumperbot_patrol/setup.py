from setuptools import find_packages, setup

package_name = 'bumperbot_patrol'

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
    maintainer='don',
    maintainer_email='dwilliestyle@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
              'bumperbot_patrol_server = \
                bumperbot_example.bumperbot_patrol.bumperbot_patrol_server:main',
            'bumperbot_patrol_client = \
                bumperbot_example.bumperbot_patrol.bumperbot_patrol_client:main',
        ],
    },
)
