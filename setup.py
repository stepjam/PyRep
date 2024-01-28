import codecs
import os
import setuptools


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


core_requirements = [
    "numpy",
    "cbor",
    "zmq"  # Not used, but installed to stop coppeliasim complaining about module
]

setuptools.setup(
    name='PyRep',
    # Version A.B.C.
    # A.B info corresponds to the CoppeliaSim version needed.
    # C info corresponds to the PyRep patch version.
    version=get_version("pyrep/__init__.py"),
    description='Python CoppeliaSim wrapper',
    author='Stephen James',
    author_email='stepjamuk@gmail.com',
    url='https://stepjam.github.io/',
    packages=setuptools.find_packages(),
    python_requires=">=3.10",
    install_requires=core_requirements,
    extras_require={
        "dev": [
            "pre-commit"
        ],
    },
)
