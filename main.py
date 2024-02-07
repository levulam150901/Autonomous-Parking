#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

from DyorBase import *
from Bug2 import *
import argparse
import sys


######################################################################################################################
def create_parser():
    parser = argparse.ArgumentParser(prog="main.py",
                                     description='''Implementation of Bug's algorithms for CoppeliaSim''',
                                     epilog=''' Author ''')

    parser.add_argument('-a', '--algorithm', choices=['BUG1', 'BUG2', 'DISTBUG'], help="Choose bug algorithm.", default='BUG2')
    parser.add_argument('-s', '--speed', type=float, help="Speed of robot wheels", default=0.5)
    parser.add_argument('-t', '--targetname', type=str, help="Name of target on scene", default='/goal')
    parser.add_argument('-b', '--botname', type=str, help="Name of bot on scene", default='/robotPose')

    return parser

######################################################################################################################
if __name__ == '__main__':
    
    parser = create_parser()
    namespace = parser.parse_args(sys.argv[1:])

    bug = None
    print (namespace)
    #namespace.algorithm = "BUG2"

    # choose algorithm
    if namespace.algorithm == "BUG1":
        bug = Bug1(target_name=namespace.targetname, bot_name=namespace.botname, wheel_speed=namespace.speed)
    elif namespace.algorithm == "BUG2":
        bug = Bug2(target_name=namespace.targetname, bot_name=namespace.botname, wheel_speed=namespace.speed)
    elif namespace.algorithm == "DISTBUG":
        bug = DistBug(target_name=namespace.targetname, bot_name=namespace.botname, wheel_speed=namespace.speed)
    else:
        print("Something goes wrong!")
        exit(-2)

    # Get bounding box position of an obstacle




    
    # Get position of obstacle vertices
    # Can be used to create an simplified Object which can be used for path planning afterwards
    """Arguments

        shapeHandle: handle of the shape. See sim.exportMesh for a usage example.

    Return values

        vertices: array of vertices
        indices: array of indices
        normals: array of normals
    """
    # list vertices, list indices, list normals = sim.getShapeMesh(int shapeHandle)



    # main loop
    bug.loop()

