import codecs
import os
import os.path
from os.path import join

from setuptools import setup

import cffi_build.cffi_build as cffi_build


def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()


def get_version(rel_path):
    for line in read(rel_path).splitlines():
        if line.startswith('__version__'):
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    else:
        raise RuntimeError("Unable to find version string.")


# Temp fix for CoppeliaSim 4.1
if 'COPPELIASIM_ROOT' not in os.environ:
    raise RuntimeError('COPPELIASIM_ROOT not defined.')

usrset_file = os.path.join(os.environ['COPPELIASIM_ROOT'], 'system', 'usrset.txt')
usrset = ''
if os.path.isfile(usrset_file):
    with open(usrset_file, 'r') as f:
        usrset = f.read()

if 'allowOldEduRelease' not in usrset:
    with open(usrset_file, 'a+') as f:
        f.write('\nallowOldEduRelease=7501\n')


setup(name='PyRep',
      # Version A.B.C.D.
      # A.B.C info corresponds to the CoppeliaSim version needed.
      # D info corresponds to the PyRep version.
      version=get_version("pyrep/__init__.py"),
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
