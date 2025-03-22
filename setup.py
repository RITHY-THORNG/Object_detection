from setuptools import setup

package_name = 'odom_pkg'

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
    maintainer='rithy',
    maintainer_email='mrrthy10a@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'odom_sub_node = odom_pkg.odom_sub_node:main', 
              'obj_detect_node= odom_pkg.obj_detect_node:main',
        ],
    },
)