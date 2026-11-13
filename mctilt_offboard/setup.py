#!/usr/bin/env python3
"""安装共享控制模块，兼容 catkin devel/install 两种脚本入口。"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['mctilt_offboard'],
    package_dir={'': 'src'},
))
