from setuptools import setup

package_name = 'unitree_camera_launch_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'unitree_camera_launch_module/ternary_text_substitution.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nick Morales',
    maintainer_email='ngmorales97@gmail.com',
    description='Launch module library for the unitree_camera package.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
