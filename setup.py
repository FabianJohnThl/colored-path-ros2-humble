from setuptools import find_packages, setup

package_name = 'col_pth'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/col_pth_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FabianJohnTHL',
    maintainer_email='fabian.john@th-luebeck.de',
    description='Package to publish a given path colored, depending on some measured values',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
                'col_pth = col_pth.col_pth:main'
        ],
    },
)
