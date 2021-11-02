import json
import argparse


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_err(msg):
    print("{}{}{}".format(bcolors.FAIL, msg, bcolors.ENDC))

def print_info(msg):
    print("{}{}{}".format(bcolors.OKBLUE, msg, bcolors.ENDC))


def print_warn(msg):
    print("{}{}{}".format(bcolors.WARNING, msg, bcolors.ENDC))


def print_ok(msg):
    print("{}{}{}".format(bcolors.OKGREEN, msg, bcolors.ENDC))
    

def load_json(file_dir):
    ret = None

    with open(file_dir, "r") as fin:
        ret = json.load(fin)

    return ret


def arg_parser():
    """Argument Parser
    Parse arguments from command line, and perform error checking
    Returns:
        An argument object which contains arguments from cmd line
    """
    parser = argparse.ArgumentParser(prog='Scene Builder')

    parser.add_argument(
        "-c, --src",
        dest="src",
        type=str,
        required=True,
        help="Input source"
    )

    parser.add_argument(
        "-o, --out",
        dest="out",
        type=str,
        default="./output",
        help="Output directory"
    )

    parser.add_argument(
        "--name",
        dest="name",
        type=str,
        default=None,
        help="Scene name"
    )

    parser.add_argument(
        "--vrgym",
        dest="vrgym",
        action="store_true",
        help="Enable VRGym"
    )

    parser.add_argument(
        "--physics",
        dest="physics",
        action="store_true",
        help="Enable Physical Properties"
    )

    parser.add_argument(
        "--gazebo",
        dest="gazebo",
        action="store_true",
        help="Enable Gazebo Output"
    )
    
    args = parser.parse_args()

    # if gazebo output is enabled, the physical properties 
    # must be enabled as well
    if args.gazebo:
        args.physics = True

    return args