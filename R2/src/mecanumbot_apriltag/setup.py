from setuptools import setup

package_name = 'mecanumbot_apriltag'

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
    maintainer='wafdan',
    maintainer_email='dantatakiyya@gmail.com',
    description='Package to calculate distance to AprilTags',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_distance_node = mecanumbot_apriltag.apriltag_distance_node:main'
        ],
    },
)
