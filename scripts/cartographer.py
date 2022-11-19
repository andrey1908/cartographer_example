import os.path as osp
from docker_helper import DockerMounts, RosDockerContainer


class CartographerMounts(DockerMounts):
    def __init__(self, files=tuple(), folders=tuple()):
        super().__init__(files=files, folders=folders)
        self.pass_cartographer_to_docker()

    def pass_cartographer_to_docker(self):
        cartographer_folder = (osp.join(osp.dirname(__file__), '../../..'))
        self.docker_cartographer_folder, volume = \
            DockerMounts.pass_folders_to_docker(cartographer_folder, '/home/docker_cartographer/catkin_ws')
        self.volume_args = self.volume_args + '-v {} '.format(volume)


class Cartographer:
    def __init__(self, cartographer_docker: RosDockerContainer):
        self.cartographer_docker = cartographer_docker
        self.transforms_topic = "/cartographer/tracked_local_transform"
        self.source_files = ['/home/docker_cartographer/catkin_ws/devel_isolated/setup.bash']

    def transforms_topic(ns='cartographer'):
        return "/{}/tracked_local_transform".format(ns)

    def run_cartographer(self, config_file, bag_files=tuple(), sleep_ms_after_first_clock=None, sleep_ms=None, online=True):
        if isinstance(bag_files, str):
            bag_files = [bag_files]
        if not online and not bag_files:
            raise RuntimeError("If offline mode is used, bag_files to process must be specified.")
        if not online and isinstance(config_file, str):
            config_file = [config_file]

        cartographer_args = ''
        if online:
            cartographer_args += 'config_filename:={} '.format(config_file)
        else:
            cartographer_args += 'config_filenames:={} '.format(','.join(config_file))
            cartographer_args += 'bag_filenames:={} '.format(','.join(bag_files))
            if sleep_ms_after_first_clock is not None:
                cartographer_args += 'sleep_ms_after_first_clock:={} '.format(sleep_ms_after_first_clock)
            if sleep_ms is not None:
                cartographer_args += 'sleep_ms:={} '.format(sleep_ms)

        if online:
            self.cartographer_docker.roslaunch_async("cartographer_example", "cartographer.launch",
                cartographer_args, session='cartographer', source_files=self.source_files)
        else:
            self.cartographer_docker.roslaunch("cartographer_example", "cartographer_offline.launch",
                cartographer_args, source_files=self.source_files)

    def stop_cartographer(self):
        self.cartographer_docker.stop_session('cartographer')
