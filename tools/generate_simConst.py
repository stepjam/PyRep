#!/usr/bin/env python

import contextlib
import importlib
import os
import os.path as osp
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

try:
    import CppHeaderParser
except ImportError:
    print('Please run following:\n\tpip install CppHeaderParser',
          file=sys.stderr)
    sys.exit(1)


def get_coppeliasim_root():
    if 'COPPELIASIM_ROOT' not in os.environ:
        raise RuntimeError('Please set env COPPELIASIM_ROOT')
    return os.environ['COPPELIASIM_ROOT']


def import_as(filename, module):
    spec = importlib.util.spec_from_file_location(
        module, filename
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_simConst_py():
    header_file = osp.join(
        get_coppeliasim_root(), 'programming/include/simConst.h'
    )
    with contextlib.redirect_stdout(sys.stderr):
        header = CppHeaderParser.CppHeader(header_file)

    assert header.classes == {}
    assert header.functions == []
    assert header.global_enums == {}

    names = []

    for enum in header.enums:
        assert enum['namespace'] == ''
        for value in enum['values']:
            names.append(value['name'])

    out_dir = tempfile.mkdtemp()

    cpp_file = osp.join(out_dir, 'generate_simConst_py.cpp')
    with open(cpp_file, 'w') as f:
        f.write('#include <iostream>\n')
        f.write('#include <simConst.h>\n')
        f.write('int main() {\n')
        for name in names:
            f.write(f'\tstd::cout << "{name} = " << {name} << std::endl;\n')
        f.write('}\n')

    out_file = osp.join(out_dir, 'generate_simConst_py')
    cmd = f'g++ {cpp_file} -o {out_file} -I{osp.dirname(header_file)}'
    subprocess.check_call(shlex.split(cmd))

    cmd = osp.join(out_dir, 'generate_simConst_py')
    code = subprocess.check_output(cmd).strip().decode()

    shutil.rmtree(out_dir)

    return code


def merge_simConst_py(code):
    official_file = osp.join(
        get_coppeliasim_root(),
        'programming/remoteApiBindings/python/python/simConst.py')
    official = import_as(official_file, 'official')

    out_dir = tempfile.mkdtemp()

    generated_file = osp.join(out_dir, 'simConst_generated.py')
    with open(generated_file, 'w') as f:
        f.write(code)
    generated = import_as(generated_file, 'generated')

    name_and_value = []
    for name in dir(generated):
        if re.match('__.*__', name):
            continue
        if name in dir(official):
            value_official = getattr(official, name)
            value_generated = getattr(generated, name)
            if value_official != value_generated:
                print(f"WARNING: The values of '{name}' are not equal: "
                      f'official={value_official}, '
                      f'generated={value_generated}', file=sys.stderr)
        else:
            name_and_value.append((name, getattr(generated, name)))

    with open(official_file) as f:
        code = f.read()

    for name, value in name_and_value:
        code += f'{name} = {value}\n'

    return code


def main():
    code = generate_simConst_py()
    code = merge_simConst_py(code)
    print(code, end='')


if __name__ == '__main__':
    main()
