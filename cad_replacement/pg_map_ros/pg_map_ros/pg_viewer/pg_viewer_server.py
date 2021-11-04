import json
import time
import threading
import os

import cv2
import igraph as ig

from tcp_server import TcpServer
from pg_viewer import PgViewer


COLOR_DICT = {
    "ConceptNode": "#0C6DEF",
    "ObjectNode": "#EF0C0C"
}


class PgViewerServer(object):

    def __init__(self, host, port, tmp_path="./tmp.png"):
        self.server_ = TcpServer(host, port)
        self.server_.launch()

        self.viewer_ = PgViewer()

        self.tmp_path_ = tmp_path
        self.quit_evt_ = threading.Event()
        self.ready_lock_ = threading.Lock()

    
    def launch(self):
        self.quit_evt_.clear()
        # updating images at backend
        pthread = threading.Thread(target=self.update_img_)
        pthread.start()

        while True:
            if os.path.exists(self.tmp_path_):
                self.ready_lock_.acquire()
                img = cv2.imread(self.tmp_path_)
                self.ready_lock_.release()
                cv2.imshow('Parse Graph',img)
            else:
                time.sleep(0.25)
            
            if cv2.waitKey(250) & 0xFF == ord('q'):
                break
        
        self.quit_evt_.set()
        self.server_.stop()
        pthread.join()

        os.system("rm -f {}".format(self.tmp_path_))
        
        
    def update_img_(self):
        for pkg in self.server_.fetch_package():
            if self.quit_evt_.is_set() or pkg is None:
                break
            
            pg_json = self.unpack_(pkg)

            self.ready_lock_.acquire()
            self.viewer_.visualize(pg_json, target=self.tmp_path_)
            self.ready_lock_.release()

    
    def unpack_(self, pkg):
        pkg = pkg.decode("utf-8")
        pg_json = json.loads(pkg)

        return pg_json


def launch_viewer_server():
    viewer = PgViewerServer(host="0.0.0.0", port=12345)
    viewer.launch()


if __name__ == "__main__":
    launch_viewer_server()