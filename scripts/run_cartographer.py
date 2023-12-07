import argparse
import time
import os
import os.path as osp
try:
    from cartographer import Cartographer
except ImportError:
    from .cartographer import Cartographer


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-file', type=str)
    parser.add_argument('--load-map', type=str)
    parser.add_argument('--save-map', type=str)

    parser.add_argument('--build', action='store_true')

    parser.add_argument('-args', '--args', action='store_true')  # workaround to pass args
    return parser


def run_cartographer():
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()

    cartographer = Cartographer()
    cartographer.create_containter()

    # build and exit
    if args.build:
        cartographer.build_cartographer()
        exit(0)

    if not args.config_file:
        raise RuntimeError("Specify config file.")

    catkin_ws_folder = osp.abspath(osp.join(osp.dirname(__file__), "../../.."))
    logs_folder = osp.abspath(osp.expanduser(osp.join(catkin_ws_folder, "cartographer_logs")))
    cartographer.set_environment_variable("artd_COL_LOG_FOLDER", cartographer.resolve(logs_folder))
    cartographer.set_environment_variable("displacement_COL_LOG_FOLDER", cartographer.resolve(logs_folder))
    cartographer.set_environment_variable("trim_loops_COL_LOG_FOLDER", cartographer.resolve(logs_folder))

    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")
    results = cartographer.run_cartographer(args.config_file,
        load_state_file=args.load_map,
        save_state_file=args.save_map, *unknown_args)

    os.makedirs(logs_folder, exist_ok=True)
    with open(osp.join(logs_folder, f"{time_str}.txt"), 'w') as f:
        f.write(results.stdout)


if __name__ == '__main__':
    run_cartographer()
