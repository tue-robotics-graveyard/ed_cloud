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

    def __init__(self):
        pass

    def __init__(self, path_server_tree, path_client_tree):
        self.parseTrees(path_server_tree, path_client_tree);

    def parseTrees(self, path_server_tree, path_client_tree):
        server_tree_file = open(path_server_tree)
        self.server_tree = json.load(server_tree_file)
        client_tree_file = open(path_client_tree)
        self.client_tree = json.load(client_tree_file)

    def testRevNumber(self):
        error = False
        print("== Begin Revision number test ==")

        if (self.server_tree["revision"] != self.client_tree["revision"]):
            print("[ERROR] Different revision numbers. Server: "
            + str(self.server_tree["revision"]) + " , Client: " +
            str(self.client_tree["revision"]))
            error = True

        if (error == False):
            print("== Entity Number Test -> Success ==")
        else:
            print("== Entity Number Test -> Failed ==")

        return error

    def testEntityNumber(self):

        print("== Begin Entity Number Test ==")

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
            print("[ERROR] These entities were not found in client: " + str(dif_server))

            error = True

        if (len(dif_server) != 0):
            print("[ERROR] These entities should not be in client: " + str(dif_client))
            error = True

        if (error == False):
            print("== Entity Number Test -> Success ==")
        else:
            print("== Entity Number Test -> Failed ==")

        return error


    def testShapesTriangles(self):
        print("== Shape Triangles Test ==")
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
                print("[ERROR] Inconsistent Triangles for instance id = " + str(id)
                + ": " + str(points) + " != " + str(entity_triangles_client[id]))
                error = True



        if (error == False):
            print("== Shape Triangles Test -> Success ==")
        else:
            print("== Shape Triangles Test -> Failed ==")

        return error

    def testShapesVertices(self):
        print("== Shape Vertices Test ==")
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
                print("[ERROR] Inconsistent Vertices for instance id = " + str(id)
                + ": " + str(points) + " != " + str(entity_vertices_client[id]))
                error = True

        if (error == False):
            print("== Shape Vertices Test -> Success ==")
        else:
            print("== Shape Vertices Test -> Failed ==")


        return error

    def testConvexHulls(self):

        print("== Convex Hull Test ==")

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
                print("[ERROR] Inconsistent Convex Hulls for instance id = " + str(id)
                + ": " + str(points) + " != " + str(entity_ch_client[id]))
                error = True

        if (error == False):
            print("== Convex Hull Test -> Success ==")
        else:
            print("== Convex Hull Test -> Failed ==")

        return error

    def testTypes(self):

        print("== Begin Entity Type Test ==")

        error = False

        entity_type_server = dict()
        entity_type_client = dict()

        for entity in self.server_tree["entities"]:
            entity_type_server[entity["id"]] = entity["type"]

        for entity in self.client_tree["entities"]:
            entity_type_client[entity["id"]] = entity["type"]

        for (id, type) in entity_type_server.items():
            if id in entity_type_client and type != entity_type_client[id]:
                print("[ERROR] Inconsistent type for instance id = " + str(id) +
                ", " + "\"" + str(type) + "\"" + " != " + "\"" + str(entity_type_client[id] + "\""))

        if (error == False):
            print("== Entity Type Test -> Success ==")
        else:
            print("== Entity Type Test -> Failed ==")

        return error
