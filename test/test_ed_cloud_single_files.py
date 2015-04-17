#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import EDSyncTester
import os, os.path
import re
from progressbar import AnimatedMarker, Bar, FormatLabel, Counter, ProgressBar, Percentage, RotatingMarker

parser = argparse.ArgumentParser(description='Test ED Cloud Syncing.')

parser.add_argument('-c', '--client-file',  type=str, nargs=1,
                    help='Client Output Directory', required=True)
parser.add_argument('-s', '--server-file',  type=str,  nargs=1,
                    help='Client Output Directory',required=True)
parser.add_argument('-p', '--pattern', type=str, nargs='?', default='output',
                    help='Pattern of the output json files. e.g. \"output-\"')
parser.add_argument('-v', '--verbose',  action='store_true',
                    help='Show Information while executing')

args = parser.parse_args()

file_name_client = args.client_file[0]
file_name_server = args.server_file[0]
pattern = args.pattern

num_files_tested = 0;
num_files_failed = 0;
failRevNum = 0;
failEntNum = 0;
failPose = 0;
failShapeTriangles = 0;
failShapeVertices = 0;
failConvexHulls = 0;
failTypes = 0;

# simple version for working with CWD

total_files = 1

if args.verbose == False:
    pbar = ProgressBar(maxval=total_files,widgets=[FormatLabel(''), Percentage(), Bar(marker=RotatingMarker())]).start()

if args.verbose == False:
    pbar.widgets[0] = FormatLabel('| Test {0}/{1} - Failed: {2} | '.format(num_files_tested, total_files, num_files_failed))

if (os.path.isfile(file_name_server)):
    success = True
    if (args.verbose == True):
        print(" Testing " + file_name_server + " vs. " + file_name_client)

    try:

        tester = EDSyncTester.EDSyncTester(file_name_server, file_name_client, args.verbose)
        num_files_tested += 1

        if (tester.testRevNumber()):
            failRevNum+= 1
            success = False

        if (tester.testEntityNumber()):
            failEntNum+= 1
            success = False

        if (tester.testTypes()):
            failTypes+= 1
            success = False

        if (tester.testPoses()):
            failPose+= 1
            success = False

        if (tester.testConvexHulls()):
            failConvexHulls+= 1
            success = False

        if (tester.testShapesVertices()):
            failShapeVertices+= 1
            success = False

        if (tester.testShapesTriangles()):
            failShapeTriangles+= 1
            success = False

        if (not success):
            num_files_failed+= 1
            if (args.verbose == True):
                print(" UNIT TEST FAILED")
        else:
            if (args.verbose == True):
                print(" UNIT TEST SUCCESSFUL")
    except ValueError as e:
        print(e.args)
        print("Error with file " + file_name_server + " or " + file_name_client)

if (args.verbose == False):
    pbar.update(num_files_tested)

print("=== General Statistics ===")
print("Total Tests: "  + str(num_files_tested))
print("Tests Failed: " + str(num_files_failed))
print("=== Detailed Statistics ===")
print("Rev. Number Test: " + str(num_files_tested - failRevNum) + "/" + str(num_files_tested))
print("Ent. Number Test: " + str(num_files_tested - failEntNum) + "/"+ str(num_files_tested))
print("Ent. Type Test:   " + str(num_files_tested - failTypes) + "/"+ str(num_files_tested))
print("Pose Test:        " + str(num_files_tested - failPose) + "/"+ str(num_files_tested))
print("Convex Hull Test: " + str(num_files_tested - failConvexHulls) + "/"+ str(num_files_tested))
print("Shape Vert. Test: " + str(num_files_tested - failShapeVertices) +"/"+  str(num_files_tested))
print("Shape Trng. Test: " + str(num_files_tested - failShapeTriangles) + "/"+ str(num_files_tested))
