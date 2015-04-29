# -*- coding: utf-8 -*-
"""
    Ed Cloud Synchornization Tester

    Javier J. Salmeron Garcia (jsalmeron2@us.es)
"""

import json
from sets import Set

class EDSyncTester:

    server_tree = []
    client_tree = []
    verbose = False

    def verbosePrint(self, *args):
        if (self.verbose == True):
            for arg in args:
               print (arg)

    def __init__(self):
        pass

    def __init__(self, path_server_tree, path_client_tree, verbose = False):
        self.parseTrees(path_server_tree, path_client_tree)
        self.verbose = verbose

    def parseTrees(self, path_server_tree, path_client_tree):
        server_tree_file = open(path_server_tree)
        self.server_tree = json.load(server_tree_file)
        client_tree_file = open(path_client_tree)
        self.client_tree = json.load(client_tree_file)

    def testRevNumber(self):
        error = False
        self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Rev. Number Test]"+ " BEGIN")

        if (self.server_tree["revision"] != self.client_tree["revision"]):
            self.verbosePrint("[ERROR] Different revision numbers. Server: "
            + str(self.server_tree["revision"]) + " , Client: " +
            str(self.client_tree["revision"]))
            error = True

        if (error == False):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Rev. Number Test]"+ " SUCCESS")
        else:
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Rev. Number Test]"+ " FAILED")

        return error

    def testPoses(self):
        error = False
        self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Pose Test]"+ " BEGIN")

        poses_server = dict()
        poses_client = dict()

        for entity in self.server_tree["entities"]:
            poses_server[entity["id"]]  = entity["pose"]

        for entity in self.client_tree["entities"]:
            poses_client[entity["id"]] = entity["pose"]

        for (id, pose) in poses_server.items():
            if (id in poses_client and sum([abs(a - b) for (a,b) in zip(pose.values(), poses_client[id].values())]) > 1e-9):
                error = True
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]" + "[Pose Test]" +
                "[ERROR]: " + "Inconsistent poses in instance with id " + str(id) + ":")
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]" + "[Pose Test]" +
                "[ERROR]: " + "Pose in server: " + str(pose))
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]" + "[Pose Test]" +
                "[ERROR]: " + "Pose in server: " + str(poses_client[id]))
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]" + "[Pose Test]" +
                "[ERROR]: " + "Difference: " + str({key: pose[key] - poses_client[id].get(key, 0) for key in pose.keys()}))


        if (error == False):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Pose Test]"+ " SUCCESS")
        else:
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Pose Test]"+ " FAILED")


        return error;

    def testEntityNumber(self):

        self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]" + "[Entity Test]" + " BEGIN")

        error = False
        entities_in_server = Set([])
        entities_in_client = Set([])

        for entity in self.server_tree["entities"]:
            entities_in_server.add(entity["id"])

        for entity in self.client_tree["entities"]:
            entities_in_client.add(entity["id"])

        dif_server = entities_in_server.difference(entities_in_client)
        dif_client = entities_in_client.difference(entities_in_server)
    

        if (len(dif_server) != 0):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]" + "[Entity Test]" +
            "[ERROR]: " +
            "These entities were not found in client: " + str(dif_server))

            error = True

        if (len(dif_client) != 0):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]" + "[Entity Test]"
            +  "[ERROR]: " +"These entities should not be in client: " + str(dif_client))
            error = True

        if (error == False):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Entity Test]"+ " SUCCESS")
        else:
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Entity Test]"+ " FAILED")

        return error


    def testShapesTriangles(self):
        self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Shape Triangle Test]"+ " BEGIN")
        error = False

        entity_triangles_server = dict()
        entity_triangles_client = dict()

        for entity in self.server_tree["entities"]:
            if ("shape" in entity):
                entity_triangles_server[entity["id"]] = [
                [entity["shape"]["vertices"][e["i1"]].values(),
                entity["shape"]["vertices"][e["i2"]].values(),
                entity["shape"]["vertices"][e["i3"]].values()]
                for e in entity["shape"]["triangles"]]
                for t in entity_triangles_server[entity["id"]]:
                    t.sort()
                entity_triangles_server[entity["id"]].sort()


        for entity in self.client_tree["entities"]:
            if ("shape" in entity):
                entity_triangles_client[entity["id"]] = [
                [entity["shape"]["vertices"][e["i1"]].values(),
                entity["shape"]["vertices"][e["i2"]].values(),
                entity["shape"]["vertices"][e["i3"]].values()]
                for e in entity["shape"]["triangles"]]
                for t in entity_triangles_client[entity["id"]]:
                    t.sort()
                entity_triangles_client[entity["id"]].sort()

        for (id, points) in entity_triangles_server.items():
            if (id in entity_triangles_client and
            points != entity_triangles_client[id]):
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Shape Triangle Test]"+
                "[ERROR] Inconsistent Triangles for instance id = " + str(id)
                + ": " + str(points) + " != " + str(entity_triangles_client[id]))
                error = True



        if (error == False):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Shape Triangle Test]"+ " SUCCESS")
        else:
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Shape Triangle Test]"+ " FAILED")

        return error

    def testShapesVertices(self):
        self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Shape Vertices Test]"+ " BEGIN")
        error = False

        entity_vertices_server = dict()
        entity_vertices_client = dict()

        for entity in self.server_tree["entities"]:
            if ("shape" in entity):
                entity_vertices_server[entity["id"]] = [(e["x"],e["y"],e["z"]) for e in entity["shape"]["vertices"]]
                entity_vertices_server[entity["id"]].sort()

        for entity in self.client_tree["entities"]:
            if ("shape" in entity):
                entity_vertices_client[entity["id"]] = [(e["x"],e["y"],e["z"]) for e in entity["shape"]["vertices"]]
                entity_vertices_client[entity["id"]].sort()

        for (id, points) in entity_vertices_server.items():
            if (id in entity_vertices_client and
            points != entity_vertices_client[id]):
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Shape Vertices Test]"+
                "[ERROR] Inconsistent Vertices for instance id = " + str(id)
                + ": " + str(points) + " != " + str(entity_vertices_client[id]))
                error = True

        if (error == False):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Shape Vertices Test]"+ " SUCCESS")
        else:
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Shape Vertices Test]"+ " FAILED")

        return error

    def testConvexHulls(self):

        self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Convex Hulls Test]"+ " BEGIN")

        entity_ch_server = dict()
        entity_ch_client = dict()

        error = False

        for entity in self.server_tree["entities"]:
            if ("convex_hull" in entity):
                entity_ch_server[entity["id"]] = [[(e["x"],e["y"]) for e in entity["convex_hull"]["points"]]]
                entity_ch_server[entity["id"]].sort()

        for entity in self.client_tree["entities"]:
            if ("convex_hull" in entity):
                entity_ch_client[entity["id"]] = [[(e["x"],e["y"]) for e in entity["convex_hull"]["points"]]]
                entity_ch_client[entity["id"]].sort()


        for (id, points) in entity_ch_server.items():
            if (id in entity_ch_client and
            points != entity_ch_client[id]):
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Convex Hulls Test]" +
                "[ERROR] Inconsistent Convex Hulls for instance id = " + str(id)
                + ": " + str(points) + " != " + str(entity_ch_client[id]))
                error = True


        if (error == False):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Convex Hulls Test]"+ " SUCCESS")
        else:
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Convex Hulls Test]"+ " FAILED")


        return error

    def testTypes(self):

        self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Types Test]"+ " BEGIN")

        error = False

        entity_type_server = dict()
        entity_type_client = dict()

        for entity in self.server_tree["entities"]:
            entity_type_server[entity["id"]] = entity["type"]

        for entity in self.client_tree["entities"]:
            entity_type_client[entity["id"]] = entity["type"]

        for (id, type) in entity_type_server.items():
            if id in entity_type_client and type != entity_type_client[id]:
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Types Test]"+
                "[ERROR] Inconsistent type for instance id = " + str(id) +
                ", " + "\"" + str(type) + "\"" + " != " + "\"" + str(entity_type_client[id] + "\""))
                error = True

        if (error == False):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Types Test]"+ " SUCCESS")
        else:
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Types Test]"+ " FAILED")


        return error

    def testMeasurements(self):

        self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Measurement Test]"+ " BEGIN")

        error = False

        entity_measurement_server = dict()
        entity_measurement_client = dict()

        for entity in self.server_tree["entities"]:
            if ("measurements" in entity):
                entity_measurement_server[entity["id"]] = entity["measurements"]

        for entity in self.client_tree["entities"]:
            if ("measurements" in entity):
                entity_measurement_client[entity["id"]] = entity["measurements"]

        for (id, measurements) in entity_measurement_server.items():
            if id in entity_measurement_client and measurements != entity_measurement_client[id]:
                self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Measurement Test]"+
                "[ERROR] Inconsistent measurement for instance id = " + str(id) +
                ", " + "\"" + str(measurements) + "\"" + " != " + "\"" + str(entity_measurement_client[id]) + "\"")
                error = True

        if (error == False):
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Measurement Test]"+ " SUCCESS")
        else:
            self.verbosePrint("[Rev. " + str(self.server_tree["revision"]) + "]"+ "[Measurement Test]"+ " FAILED")


        return error

