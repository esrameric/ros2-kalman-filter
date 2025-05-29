from setuptools import setup
import os

package_name = 'kalman_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (package_name, [
    		'kalman_pkg/logger_node.py',
        	'kalman_pkg/publisher.py',
     		'kalman_pkg/kalman_node.py',
        	'kalman_pkg/filter_params.py'
       ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esrameric',
    maintainer_email='mericesra01@gmail.com',
    description='Kalman filtresiyle ölçüm verisi işleyen ROS 2 paketi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = kalman_pkg.publisher:main',
            'kalman_node = kalman_pkg.kalman_node:main',
            'logger_node = kalman_pkg.logger_node:main',
        ],
    },
)
