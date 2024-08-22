from setuptools import setup, find_packages

setup(name = 'picarx',
  #packages=['picarx','picarx.interfaces', 'picarx.drivers', 'picarx.emulators'],
  packages=find_packages('picarx'),
  package_dir={'': 'src'},
  install_requires=['incremental<=22.10.0','watchdog<=4.0.0', 'smbus2', 'pyyaml', 'twisted<=22.10.0']
)
