import json
import time
import threading
import os

import cv2
import igraph as ig

from tcp_server import TcpServer
from pg_parser import PgParser


COLOR_DICT = {
    "ConceptNode": "#0C6DEF",
    "ObjectNode": "#EF0C0C"
}


class PgViewer(object):

    def __init__(self):
        self.parser_ = PgParser()


    def show(self, pg_file):
        pg_json = self.load_jsonfile_(pg_file)

        self.visualize(pg_json)


    def save2img(self, pg_file, output_file):
        pg_json = self.load_jsonfile_(pg_file)

        self.visualize(pg_json, target=output_file)


    def visualize(self, pg_json, target=None):
        g, root = self.parser_.parse(pg_json)

        layout = g.layout("rt", root=root)

        visual_style = {
            "edge_width": 1,
            # "edge_color": [],
            # "edge_curved": [],
            "edge_arrow_size": 1,
            # "edge_arrow_width": 2,
            # "vertex_label_angle": [],
            "vertex_label_dist": 1.1,
            "vertex_label_size": 10,
            "vertex_label": self.generate_vertex_labels_(g),
            "vertex_size": 40,
            "vertex_color": self.generate_vertex_color_(g),
            "vertex_shape": self.generate_vertex_shape_(g),
            # "autocurve": False,
            "bbox": (1000, 400),
            "margin": 50,
            "layout": layout
        }
        
        if target is None:
            ig.plot(g, **visual_style)
        else:
            ig.plot(g, target=target, **visual_style)

    
    def generate_vertex_color_(self, g):
        # colors = [COLOR_DICT[t] for t in g.vs["type"]]
        colors = ["red" for t in g.vs["type"]]
        return colors

    
    def generate_vertex_shape_(self, g):
        # known shapes
        # circle, circular, diamond, box, rectangle, arrow, arrow-down, arrow-up
        shapes = ["circle" for t in g.vs["type"]]
        return shapes


    def generate_vertex_labels_(self, g):
        labels = []
        for i in range(g.vcount()):
            labels.append(g.vs[i]["label"] + "\nID: " + str(g.vs[i]["id"]))
        return labels


    def load_jsonfile_(self, json_file):
        jfile = open(json_file, "r")
        pg_json = json.load(jfile)
        jfile.close()

        return pg_json