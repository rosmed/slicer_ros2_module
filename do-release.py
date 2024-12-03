#!/usr/bin/python

import argparse
import sys
import os
import subprocess

#import lsb_release
import distro

def execute_command(directory, cmd):
    owd = os.getcwd()
    os.chdir(directory)
    print(f'>>>> Execute [{cmd}] in directory {directory}')
    result = subprocess.run(cmd, shell = True, stdout = subprocess.PIPE)
    os.chdir(owd)
    res = result.stdout.decode('utf-8').rstrip()
    print(f'<<<< [{cmd}] done: {res}')
    return res

parser = argparse.ArgumentParser (
    prog = 'SlicerROS2 binary release',
    description = 'Create a tgz file to be uploaded on GitHub in release assets')

parser.add_argument('-b', '--slicer-build-directory',
                    required = True)
parser.add_argument('-v', '--slicer-ros2-version',
                    required = True)

args = parser.parse_args()

slicer_bin = args.slicer_build_directory

ros_distro = os.environ.get('ROS_DISTRO', '')
if ros_distro == '':
    print(f'!!!! Environment variable ROS_DISTRO is not defined, make sure you source your /opt/ros setup.bash')
    sys.exit()

slicer_tgz_files = [f for f in os.listdir(slicer_bin) if (f.startswith('Slicer-') and f.endswith('.tar.gz'))]
nb_files = len(slicer_tgz_files)
if nb_files == 0:
    print('---- No tgz file found, assuming we need to run cmake package')
    result = execute_command(slicer_bin, 'make package')
    print(result)
    slicer_tgz_files = [f for f in os.listdir(slicer_bin) if f.startswith(f'Slicer-{slicer_version}')]
    nb_files = len(slicer_tgz_files)

if nb_files == 0:
    print('!!!! Can not find nor generate Slicer package file')
    sys.exit()

if nb_files != 1:
    print('!!!! Found more that one Slicer*.tar.gz files, maybe remove old SlicerROS2 file?')
    sys.exit()

slicer_tgz = slicer_tgz_files[0]
print(f'---- Found Slicer tgz file: {slicer_tgz}')

slicer_dir = slicer_tgz.replace('.tar.gz', '')

# build new name

#lsb = lsb_release.get_distro_information()
# sr2_dir = f'{slicer_dir}-SlicerROS2-{args.slicer_ros2_version}-{lsb["ID"]}-{lsb["RELEASE"]}-{ros_distro}'

lsb = distro.lsb_release_info()
sr2_dir = f'{slicer_dir}-SlicerROS2-{args.slicer_ros2_version}-{lsb["distributor_id"]}-{lsb["release"]}-{ros_distro}'

# extract install for Slicer only
execute_command(slicer_bin, f'tar zxf {slicer_tgz}')

# add some files
execute_command(slicer_bin, f'cp vtkSlicerROS2Module*.txt {slicer_dir}')

module_subdir = 'lib/Slicer-5.6/qt-loadable-modules'
execute_command(slicer_bin, f'cp {module_subdir}/*ROS2* {slicer_dir}/{module_subdir}')

# cleanup
execute_command(slicer_bin, 'find . -name *.so -exec strip {} \\;')
execute_command(slicer_bin, 'find . -name *.pyc -exec rm {} \\;')

# build new tar file
execute_command(slicer_bin, f'mv {slicer_dir} {sr2_dir}')
execute_command(slicer_bin, f'tar cf {sr2_dir}.tar {sr2_dir}')
execute_command(slicer_bin, f'gzip --best {sr2_dir}.tar')
