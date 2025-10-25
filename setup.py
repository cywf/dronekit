"""Setup script for DroneKit Python library."""

from setuptools import setup, find_packages
import os

# Read the README file
def read_file(filename):
    """Read a file and return its contents."""
    filepath = os.path.join(os.path.dirname(__file__), filename)
    if os.path.exists(filepath):
        with open(filepath, 'r', encoding='utf-8') as f:
            return f.read()
    return ''

setup(
    name='dronekit-custom',
    version='0.1.0',
    description='Python library for drone programming using MAVLink protocol',
    long_description=read_file('README.md'),
    long_description_content_type='text/markdown',
    author='Ky1o P',
    author_email='',
    url='https://github.com/cywf/dronekit',
    license='MIT',
    packages=find_packages(exclude=['tests', 'tests.*', 'examples', 'examples.*']),
    install_requires=[
        'pymavlink>=2.4.37',
        'pyserial>=3.5',
        'monotonic>=1.6',
    ],
    extras_require={
        'dev': [
            'pytest>=7.0.0',
            'pytest-cov>=3.0.0',
            'pytest-mock>=3.10.0',
            'black>=22.0.0',
            'flake8>=4.0.0',
            'mypy>=0.950',
        ],
    },
    python_requires='>=3.7',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: System :: Hardware',
    ],
    keywords='drone mavlink ardupilot px4 uav',
)
