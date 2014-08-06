import platform
import os
from pprint import pprint
import sys

from paver.easy import task, needs, path, sh, cmdopts, options
from paver.setuputils import setup, find_package_data

import version
sys.path.append(path('.').abspath())
from dmf_control_board_rpc import get_sketch_directory, package_path
from arduino_rpc.proto import CodeGenerator


dmf_control_board_rpc_files = find_package_data(package=
                                                'dmf_control_board_rpc',
                                                where= 'dmf_control_board_rpc',
                                                only_in_packages=False)
pprint(dmf_control_board_rpc_files)

PROTO_PREFIX = 'commands'

DEFAULT_ARDUINO_BOARDS = ['mega2560']

setup(name='wheeler.dmf_control_board_rpc',
      version=version.getVersion(),
      description='Arduino RPC node packaged as Python package.',
      author='Christian Fobel',
      author_email='christian@fobel.net',
      url='http://github.com/wheeler-microfluidics/dmf_control_board_rpc.git',
      license='GPLv2',
      packages=['dmf_control_board_rpc'],
      package_data=dmf_control_board_rpc_files)


@task
@cmdopts([('disable_i2c', 'd', 'Disable I2C communication.')])
def generate_command_code():
    code_generator = CodeGenerator(get_sketch_directory().joinpath('Node.h'),
                                   disable_i2c=getattr(options, 'disable_i2c',
                                                       False))

    definition_str = code_generator.get_protobuf_definitions()
    output_dir = package_path().joinpath('protobuf').abspath()
    output_file = output_dir.joinpath('%s.proto' % PROTO_PREFIX)
    with output_file.open('wb') as output:
        output.write(definition_str)

    header_str = code_generator.get_command_processor_header()
    output_dir = get_sketch_directory()
    output_file = output_dir.joinpath('NodeCommandProcessor.h')
    with output_file.open('wb') as output:
        output.write(header_str)


@task
# Generate protocol buffer request and response definitions, implementing an
# RPC API using the union message pattern suggested in the [`nanopb`][1]
# examples.
#
# [1]: https://code.google.com/p/nanopb/source/browse/examples/using_union_messages/README.txt
@needs('generate_command_code')
def generate_nanopb_code():
    nanopb_home = package_path().joinpath('libs', 'nanopb').abspath()
    output_dir = package_path().joinpath('protobuf').abspath()
    working_dir = os.getcwd()
    try:
        os.chdir(output_dir)
        print os.getcwd()
        if platform.platform().startswith('Windows'):
            #sh('protoc --nanopb_out=nano --python_out=py %s.proto' %
               #PROTO_PREFIX)
            raise RuntimeError('Windows compilation of protocol buffer '
                               'commands not currently supported.')
        else:
            #sh('./protoc.sh "%s" %s.proto .' % (nanopb_home, PROTO_PREFIX))
            sh('./protoc.sh %s %s.proto . ; cd nano ; rename -f \'s/\.pb/_pb/g\' *.* ; sed \'s/\.pb/_pb/g\' -i *.h *.c ; mv *.* %s' % (
                nanopb_home, PROTO_PREFIX, get_sketch_directory()))
    finally:
        os.chdir(working_dir)


@task
@needs('generate_nanopb_code')
def copy_nanopb_python_module():
    code_dir = package_path().joinpath('protobuf', 'py').abspath()
    output_dir = package_path().abspath()
    protobuf_commands_file = list(code_dir.files('*_pb2.py'))[0]
    protobuf_commands_file.copy(output_dir.joinpath('protobuf_commands.py'))


@task
@needs('copy_nanopb_python_module', 'generate_command_code')
@cmdopts([('sconsflags=', 'f', 'Flags to pass to SCons.'),
          ('boards=', 'b', 'Comma-separated list of board names to compile '
           'for (e.g., `mega2560`).')])
def build_firmware():
    scons_flags = getattr(options, 'sconsflags', '')
    boards = [b.strip() for b in getattr(options, 'boards', '').split(',')
              if b.strip()]
    if not boards:
        boards = DEFAULT_ARDUINO_BOARDS
    for board in boards:
        # Compile firmware once for each specified board.
        sh('scons %s ARDUINO_BOARD="%s"' % (scons_flags, board))


@task
@needs('generate_setup', 'minilib', 'build_firmware',
       'setuptools.command.sdist')
def sdist():
    """Overrides sdist to make sure that our setup.py is generated."""
    pass
