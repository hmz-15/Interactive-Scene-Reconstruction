import igraph as ig
import cv2


from utils import load_json


COLOR_DICT = {
    "ConceptNode": "#f2c2cb",
    "ObjectNode": "#f2c2cb"
}


class ParseGraph(object):

    def __init__(self, json_file):
        self.g_, self.root_id_ = self.parse_(load_json(json_file))


    def __str__(self):
        ret = ""

        ret += "Root: {}\n".format(self.root_id_)
        ret += self.g_.__str__()

        return ret


    def get_node(self, node_idx):
        return self.g_.vs[node_idx]

    
    def get_parent(self, node_idx):
        """
        Get the parent node given a node index

        @param node_idx: the index of the node
        @return (igraph.Vertex) the parent node of the given node
        """
        parent = self.g_.vs[node_idx].predecessors()

        assert(len(parent) <= 1)

        if len(parent) == 1:
            return parent[0]
        else:
            return None

    
    def get_root_idx(self):
        return self.root_id_


    def get_children_idx(self, parent_idx):
        """
        Get children IDs
        """
        children = []

        for child in self.g_.vs[parent_idx].successors():
            children.append(child.index)

        return children


    def get_parent_idx(self, node_idx):
        parent = self.g_.vs[node_idx].predecessors()

        assert(len(parent) <= 1)

        if len(parent) == 1:
            return parent[0].index
        else:
            return None

    
    def show(self):
        self.visualize()


    def save2img(self, output_file):
        self.visualize(target=output_file)


    def visualize(self, target=None):
        g, root = self.g_, self.root_id_

        layout = g.layout("rt", root=root)

        visual_style = {
            "edge_width": 2,
            # "edge_color": [],
            # "edge_curved": [],
            "edge_arrow_size": 1,
            # "edge_arrow_width": 2,
            # "vertex_label_angle": [],
            "vertex_label_dist": 1.1,
            "vertex_label_size": 15,
            "vertex_label": self.generate_vertex_labels_(g),
            "vertex_size": 30,
            "vertex_color": self.generate_vertex_color_(g),
            "vertex_shape": self.generate_vertex_shape_(g),
            # "autocurve": False,
            "bbox": (1000, 500),
            "margin": 50,
            "layout": layout
        }
        
        if target is None:
            ig.plot(g, **visual_style)
        else:
            ig.plot(g, target=target, **visual_style)

    
    def generate_vertex_color_(self, g):
        colors = [COLOR_DICT[t] for t in g.vs["type"]]
        # colors = ["red" for t in g.vs["type"]]
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


    def parse_(self, pg_json):
        # re-id the graph since igraph only support continous indexing from 0-N
        self.re_id_(pg_json)

        edges = []
        for e in pg_json["edges"]:
            edges.append( (e["src_id"], e["dst_id"]) )
        
        g = ig.Graph(edges, directed=True)

        root = int(pg_json["root_id"])
        
        nodes = pg_json["nodes"]
        for node in nodes:
            if node["node_type"] == "ConceptNode":
                self.update_concept_node_(g, node)
            elif node["node_type"] == "ObjectNode":
                self.update_object_node_(g, node)

        return g, root

    
    def re_id_(self, pg_json):
        cnt = 0
        id_dict = {}

        for e in pg_json["edges"]:
            src = e["src_id"]
            dst = e["dst_id"]
            if src not in id_dict:
                id_dict[src] = cnt
                cnt += 1
            if dst not in id_dict:
                id_dict[dst] = cnt
                cnt += 1
            e["src_id"] = id_dict[src]
            e["dst_id"] = id_dict[dst]
        
        for node in pg_json["nodes"]:
            node["nid"] = id_dict[node["id"]]
        
        pg_json["root_id"] = id_dict[pg_json["root_id"]]


    def update_concept_node_(self, g, node):
        nid = node["nid"]
        g.vs[nid]["id"] = node["id"]
        g.vs[nid]["label"] = node["concept"]
        g.vs[nid]["type"] = "ConceptNode"


    def update_object_node_(self, g, node):
        nid = node["nid"]
        g.vs[nid]["id"] = node["id"]
        g.vs[nid]["cad_id"] = node["cad_id"]
        g.vs[nid]["label"] = node["label"]
        g.vs[nid]["scale"] = node["scale"]
        g.vs[nid]["type"] = "ObjectNode"
        g.vs[nid]["position"] = node["position"]
        g.vs[nid]["orientation"] = node["orientation"]
        g.vs[nid]["box"] = node["box"]
