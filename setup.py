#!/usr/bin/env python

from setuptools import setup

package_name = 'rqt_robot_monitor'
version = '1.0.5'

setup(
    name = package_name,
    version = version,
    packages = [package_name],
    package_dir = {'': 'src'},
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource',
            ['resource/robotmonitor_mainwidget.ui']),
        ('share/' + package_name + '/resource',
            ['resource/timelinepane.ui'])
    ],
    install_requires = ['setuptools'],
    zip_safe = True,
    author = "Austin Hendrix, Isaac Saito, Ze'Ev Klapow, Josh Faust",
    maintainer = "Aaron Blasdel, Arne Hitzmann, Austin Hendrix",
    maintainer_email = "ablasdel@gmail.com, arne.hitzmann@gmail.com, namniart@gmail.com",
    keywords = ["ROS 2"],
    classifiers = [
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description = (
        'rqt_robot_monitor displays diagnostics_agg topics messages that \
        are published by "http://www.ros.org/wiki/diagnostic_aggregator" (diagnostic_aggregator). \
        rqt_robot_monitor is a direct port to rqt of \
        "http://www.ros.org/wiki/robot_monitor" (robot_monitor).'
    ),
    license = 'BSD',
    tests_require = ['pytest'],
    entry_points = {
        'console_scripts': [
            'rqt_robot_monitor = rqt_robot_monitor.robot_monitor:main'
        ]
    }



)
