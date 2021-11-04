import json
import igraph as ig


class PgParser(object):

    def __init__(self):
        pass

    def parse(self, pg_json):
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
        g.vs[nid]["type"] = "ObjectNode"
        g.vs[nid]["position"] = node["position"]
        g.vs[nid]["orientation"] = node["orientation"]
        g.vs[nid]["box"] = node["box"]
        g.vs[nid]["ious"] = node["ious"]
