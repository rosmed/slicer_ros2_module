#!/usr/bin/python3

import argparse
import sys

import rclpy

def ROS2_to_vtkObject(_message, _class_name, _directory):
    [package, namespace, message] = _message.split('/')
    with open(_directory + '/' + _class_name + '.h', 'w') as h:
        h.write('#ifndef _' + _class_name + '_h\n')
        h.write('#define _' + _class_name + '_h\n')
        h.write('#include <vtkObject.h>\n')
        h.write('class ' + _class_name + ': public vtkObject {\n')
        h.write('public:\n')
        h.write('  vtkTypeMacro(' + _class_name + ', vtkObject);\n')
        h.write('  static ' + _class_name + ' * New(void);\n')
        h.write('protected:\n')
        h.write('  ' + _class_name + '() {};\n')
        h.write('  ~' + _class_name + '() {};\n')
        h.write('};\n')
        h.write('#endif\n')

    with open(_directory + '/' + _class_name + '.cxx', 'w') as cxx:
        cxx.write('#include <vtkObjectFactory.h>\n')
        cxx.write('#include <' + _class_name + '.h>\n')
        cxx.write('vtkStandardNewMacro(' + _class_name + ');\n')
        cxx.write('\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--message', type = str, required=True,
                        help = 'ROS message type.  For example \"geometry_msgs/msg/PointStamped\"')
    parser.add_argument('-c', '--class-name', type = str, required = True)
    parser.add_argument('-d', '--directory', type = str, required = True)
    args = parser.parse_args(sys.argv[1:])
    application = ROS2_to_vtkObject(args.message, args.class_name, args.directory)
