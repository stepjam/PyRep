from distutils.core import setup
import cffi_build.cffi_build as cffi_build
from os.path import join

setup(name='PyRep',
      version='1.2',
      description='Python CoppeliaSim wrapper',
      author='Stephen James',
      author_email='slj12@ic.ac.uk',
      url='https://www.doc.ic.ac.uk/~slj12',
      packages=['pyrep',
                'pyrep.backend',
                'pyrep.objects',
                'pyrep.sensors',
                'pyrep.robots',
                'pyrep.robots.arms',
                'pyrep.robots.end_effectors',
                'pyrep.robots.mobiles',
                'pyrep.robots.configuration_paths',
                'pyrep.textures',
                'pyrep.misc',
                ],
      ext_modules=[cffi_build.ffibuilder.distutils_extension(
          join('build', 'pyrep', 'backend'))],
      )
