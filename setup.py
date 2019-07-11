from distutils.core import setup
import cffi_build.cffi_build as cffi_build#
from shutil import copyfile
import os

vrep_root = os.environ['VREP_ROOT']
lua_script_fname = 'vrepAddOnScript_PyRep.lua'
copyfile(os.path.join('pyrep/backend', lua_script_fname), os.path.join(vrep_root, lua_script_fname))

setup(name='PyRep',
      version='1.0',
      description='Python V-Rep wrapper',
      author='Stephen James',
      author_email='slj12@ic.ac.uk',
      url='https://www.doc.ic.ac.uk/~slj12',
      packages=['pyrep',
                'pyrep.backend',
                'pyrep.objects',
                'pyrep.robots',
                'pyrep.robots.arms',
                'pyrep.robots.end_effectors',
                'pyrep.textures'
                ],
      ext_modules=[cffi_build.ffibuilder.distutils_extension(
          os.path.join('build', 'pyrep', 'backend'))],
      )
