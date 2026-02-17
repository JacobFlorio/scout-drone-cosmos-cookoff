from setuptools import setup

package_name = 'cosmos_agent'

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
    maintainer='Jacob Florio',
    maintainer_email='jacobflorio039@gmail.com',
    description='ROS2 node for Cosmos Reason 2 threat reasoning on SIYI A8 camera frames',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'threat_reasoner = cosmos_agent.threat_reasoner:main',
        ],
    },
)
