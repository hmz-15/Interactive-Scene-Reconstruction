import argparse

from pg_viewer import PgViewer
from pg_viewer_server import PgViewerServer


def arg_parser():
    """Argument Parser
    Parse arguments from command line, and perform error checking
    Returns:
        An argument object which contains arguments from cmd line
    """

    parser = argparse.ArgumentParser(prog='PG Viewer')

    parser.add_argument(
        "--mode",
        dest="mode",
        type=str,
        default="viewer",
        help="Launch mode [server/viewer]"
    )

    parser.add_argument(
        "--host",
        dest="host",
        type=str,
        default="0.0.0.0",
        help="IP address of the server"
    )

    parser.add_argument(
        "--port",
        dest="port",
        type=int,
        default=12345,
        help="Port of viewer service on the server"
    )

    parser.add_argument(
        "-c", "--src",
        dest="src",
        type=str,
        required=False,
        help="Parse graph JSON file"
    )
    
    args = parser.parse_args()

    if args.mode == "viewer":
        try:
            assert(args.src is not None)
        except:
            print("Need to specify the directory of the parse graph JSON file")
            print("-c <pg-json-dir>")
            exit(1)

    return args


if __name__ == "__main__":
    args = arg_parser()
    
    if args.mode == "server":
        viewer = PgViewerServer(host=args.host, port=args.port)
        viewer.launch()
    else:
        viewer = PgViewer()
        viewer.show(args.src)