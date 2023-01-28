import argparse
import time
import os
import os.path as osp
from .cartographer import Cartographer, CartographerMounts


def build_parser():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('-l', '--localization', action='store_true')
    mode.add_argument('-m', '--mapping', action='store_true')
    mode.add_argument('-o', '--odometry', action='store_true')
    parser.add_argument('--load-map', type=str)
    parser.add_argument('--save-map', type=str)

    parser.add_argument('--build', action='store_true')
    return parser


def check_arguments(args):
    if args.build:
        return
    if not args.localization and not args.mapping and not args.odometry:
        raise RuntimeError("Specify cartographer running mode")


def run_cartographer():
    parser = build_parser()
    args = parser.parse_args()
    check_arguments(args)

    cartographer = Cartographer('cartographer:latest', 'cartographer')
    cartographer.create_containter()

    # build and exit
    if args.build:
        cartographer.build_cartographer()
        exit(0)

    catkin_ws_folder = osp.abspath(osp.join(osp.dirname(__file__), "../../.."))
    docker_catkin_ws_folder = "/home/docker_cartographer/catkin_ws"
    if args.localization:
        config_filename = osp.join(docker_catkin_ws_folder,
            "src/cartographer_example/config/husky_localization.lua")
    if args.mapping:
        config_filename = osp.join(docker_catkin_ws_folder,
            "src/cartographer_example/config/husky_mapping.lua")
    if args.odometry:
        config_filename = osp.join(docker_catkin_ws_folder,
            "src/cartographer_example/config/husky_odometry.lua")

    if args.load_map:
        load_map_filename = osp.abspath(osp.expanduser(args.load_map))
        if not load_map_filename.startswith(catkin_ws_folder + '/'):
            raise RuntimeError(f"Load map file {args.load_map} should be in "
                f"{catkin_ws_folder} folder")
        load_state_filename = osp.join(docker_catkin_ws_folder,
            osp.relpath(load_map_filename, catkin_ws_folder))
    else:
        load_state_filename = None

    if args.save_map:
        save_map_filename = osp.abspath(osp.expanduser(args.save_map))
        if not save_map_filename.startswith(catkin_ws_folder + '/'):
            raise RuntimeError(f"Save map file {args.save_map} should be in "
                f"{catkin_ws_folder} folder")
        save_state_filename = osp.join(docker_catkin_ws_folder,
            osp.relpath(save_map_filename, catkin_ws_folder))
    else:
        save_state_filename = None

    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")
    results = cartographer.run_cartographer(config_filename,
        load_state_filename=load_state_filename,
        save_state_filename=save_state_filename)

    logs_folder = osp.abspath(osp.expanduser(osp.join(catkin_ws_folder, "cartographer_logs")))
    os.makedirs(logs_folder, exist_ok=True)
    with open(osp.join(logs_folder, f"{time_str}.txt"), 'w') as f:
        f.write(results.stdout)
