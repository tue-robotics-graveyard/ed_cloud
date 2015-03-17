#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import EDSyncTester
import os, os.path
import re

parser = argparse.ArgumentParser(description='Test ED Cloud Syncing.')

parser.add_argument('-c', '--client-dir',  type=str, nargs=1,
                    help='Client Output Directory', required=True)
parser.add_argument('-s', '--server-dir',  type=str,  nargs=1,
                    help='Client Output Directory',required=True)
parser.add_argument('-p', '--pattern', type=str, nargs='?', default='output',
                    help='Pattern of the output json files. e.g. \"output-\"')
parser.add_argument('-v', '--verbose',
                    help='Show Information while executing')

args = parser.parse_args()

client_dir = args.client_dir[0]
server_dir = args.server_dir[0]
pattern = args.pattern

num_files_tested = 0;
num_files_failed = 0;
failRevNum = 0;
failEntNum = 0;
failShapeTriangles = 0;
failShapeVertices = 0;
failConvexHulls = 0;

# simple version for working with CWD

for i in os.listdir(client_dir):
    file_name_client = str(i)
    m = re.match(".*-(\d+)\.json", file_name_client)
    file_name_server = pattern + "-server-" + str(m.group(1)) + ".json"

    if (os.path.isfile(server_dir + file_name_server)):
        success = True
        num_files_tested += 1
        print("Testing " + file_name_server + " vs. " + file_name_client)
        tester = EDSyncTester.EDSyncTester(server_dir + file_name_server, client_dir + file_name_client)

        if (tester.testRevNumber()):
            failRevNum+= 1
            success = False

        if (tester.testEntityNumber()):
            failEntNum+= 1
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

print("=== General Statistics ===")
print("Total Tests: "  + str(num_files_tested))
print("Tests Failed: " + str(num_files_failed))
print("=== Detailed Statistics ===")
print("Rev. Number Test: " + str(num_files_tested - failRevNum) + "/" + str(num_files_tested))
print("Ent. Number Test: " + str(num_files_tested - failEntNum) + "/"+ str(num_files_tested))
print("Convex Hull Test: " + str(num_files_tested - failConvexHulls) + "/"+ str(num_files_tested))
print("Shape Vert. Test: " + str(num_files_tested - failShapeVertices) +"/"+  str(num_files_tested))
print("Shape Trng. Test: " + str(num_files_tested - failShapeTriangles) + "/"+ str(num_files_tested))
