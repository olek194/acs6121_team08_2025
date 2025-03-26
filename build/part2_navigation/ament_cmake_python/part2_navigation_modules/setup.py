from setuptools import find_packages
from setuptools import setup

setup(
    name='part2_navigation_modules',
    version='0.0.0',
    packages=find_packages(
        include=('part2_navigation_modules', 'part2_navigation_modules.*')),
)
