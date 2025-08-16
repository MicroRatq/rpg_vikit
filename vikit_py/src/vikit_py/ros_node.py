#!/usr/bin/env python3

import os
import subprocess

class RosNode:
    def __init__(self, package, executable):
        self._package = package
        self._executable = executable
        self._param_string = ''
    
    def add_parameters(self, namespace, parameter_dictionary):
        for key in parameter_dictionary.keys():
            if type(parameter_dictionary[key]) is dict:
                self.add_parameters(namespace+key+'/', parameter_dictionary[key])
            else:
                self._param_string += ' --ros-args -p '+namespace+key+':='+str(parameter_dictionary[key])
        
    def run(self, parameter_dictionary, namespace=''):
        self.add_parameters(namespace, parameter_dictionary)
        print('Starting ROS2 node with parameters: '+self._param_string)
        
        # Use ros2 run instead of rosrun
        cmd = 'ros2 run ' + self._package + ' ' + self._executable + ' ' + self._param_string
        subprocess.run(cmd, shell=True)
        print('ROS2 node finished processing.')