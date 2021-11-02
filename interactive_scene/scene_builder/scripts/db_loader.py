import random

from mesh_object import MeshObject


class DbLoader(object):

    def __init__(self, db_dir, config_file):
        self.db_ = {}
        self.categories = []
        
        self.init_db_(db_dir, config_file)

    
    def __str__(self):
        ret = ""

        ret += "3D CAD Model Database Info:\n"
        
        ret += " - Categories:\n"
        for k, v in self.db_.items():
            ret += "    * {}: {} items\n".format(k, len(v))

        return ret


    def random_pick(self, category):
        """
        Randomly pick up an object

        @param category (string): category name
        @return (string) obj ID in shapenetsem
        """
        try:
            assert(category in self.db_)
        except:
            print("[ERROR] Category `{}` not in the database".format(category))
            raise

        return random.sample(self.db_[category].keys(), 1)[0]


    def init_db_(self, db_dir, config_file):
        fconfig = open(config_file, "r")
        
        # skip the first line
        fconfig.readline()

        for line in fconfig:
            obj = MeshObject(line)
            
            if obj.category not in self.db_:
                self.db_[obj.category] = {}
            
            try:
                assert(obj.instance_id not in self.db_[obj.category])
            except:
                print("[ERROR] duplicated instance ID: {}".format(obj.instance_id))
            
            self.db_[obj.category][obj.instance_id] = obj

        fconfig.close()

        self.categories = [k for k, v in self.db_.items()]


if __name__ == "__main__":
    db_dir = "../assets/meshes/rigid/"
    config_file = db_dir + "cad_models_rigid.csv"

    db_loader = DbLoader(db_dir, config_file)
    print(db_loader)