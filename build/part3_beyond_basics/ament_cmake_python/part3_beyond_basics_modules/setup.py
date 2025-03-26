from setuptools import find_packages
from setuptools import setup

setup(
    name='part3_beyond_basics_modules',
    version='0.0.0',
    packages=find_packages(
        include=('part3_beyond_basics_modules', 'part3_beyond_basics_modules.*')),
)
