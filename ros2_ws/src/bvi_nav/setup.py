from setuptools import find_packages, setup

package_name = 'bvi_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bvi_demo.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/bvi.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav Kashyap',
    maintainer_email='abhinav23022@iiitd.ac.in',
    description='Backward Value Iteration on a 2-D grid with RViz2 visualisation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bvi_node = bvi_nav.bvi_node:main',
            'bvi_demo = bvi_nav.demo_cli:main',
        ],
    },
)
