## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
   2 
   3 from distutils.core import setup
   4 from catkin_pkg.python_setup import generate_distutils_setup
   5 
   6 # fetch values from package.xml
   7 setup_args = generate_distutils_setup(
   8     packages=['comms_messenger'],
   9     package_dir={'': 'src/comms_messenger'},
  10 )
  11 
  12 setup(**setup_args)
