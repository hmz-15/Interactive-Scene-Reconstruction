# pg_map_ros

ROS package for creating a parse graph map.


## 1. Dependencies for pg_viewer

- PyCairo
- python-igraph

Try following command to install the dependencies

```bash
pip install python-igraph
pip install PyCairo
```

## 2. Usage

### 2.1 pg_map_ros Usage

Example could be found at [here](tests/)


### 2.2 pg_viewer

The [pg_viewer](pg_viewer/) is a independent tool for visualizing the parse graph.

To visualize dumped parse graph JSON file, try
```bash
python pg_viewer/launch_viewer.py -c <pg-json-file-dir>
```

To visualize the parse graph in real-time (and asynchronously), you need to launch a pg_viewer server
```bash
python pg_viewer/launch_viewer.py --mode=server --host=<host-ip> --port=<port>
```
Then send the parse graph to the server via socket, example could be found [here](tests/test_socket_client.cpp).
