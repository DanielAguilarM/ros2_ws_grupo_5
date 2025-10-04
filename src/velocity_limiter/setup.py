from setuptools import setup
package_name = 'velocity_limiter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/velocity_limiter.launch.py',
                                               'launch/chain_example.launch.py']),
        ('share/' + package_name + '/params', ['params/velocity_limiter.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Aguilar',
    maintainer_email='example@example.com',
    description='Intermediate velocity clamp policy for ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'velocity_limiter_node = velocity_limiter.velocity_limiter_node:main',
        ],
    },
)

