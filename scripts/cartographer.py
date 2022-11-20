import os.path as osp
from docker_helper import DockerMounts, RosDockerContainer


class CartographerMounts(DockerMounts):
    def __init__(self, files=tuple(), folders=tuple()):
        super().__init__(files=files, folders=folders)
        self.pass_cartographer_to_docker()

    def pass_cartographer_to_docker(self):
        cartographer_ws_folder = (osp.join(osp.dirname(__file__), '../../..'))
        self.docker_cartographer_ws_folder, volume = \
            DockerMounts.pass_folders_to_docker(cartographer_ws_folder, '/home/docker_cartographer/catkin_ws')
        self.volume_args = self.volume_args + f'-v {volume} '


class Cartographer(RosDockerContainer):
    def __init__(self, image_name, container_name, user_name=None):
        super().__init__(image_name, container_name, user_name=user_name)
        self.local_odometry_topic = '/cartographer/tracked_local_odometry'
        self.source_files = ['/home/docker_cartographer/catkin_ws/devel_isolated/setup.bash']

    def create_containter(self, net='host', cartographer_mounts: CartographerMounts=None):
        if cartographer_mounts is None:
            cartographer_mounts = CartographerMounts()
        super().create_containter(net=net, docker_mounts=cartographer_mounts)

    def run_cartographer(self, config_filename, load_state_filename='""'):
        result = self.roslaunch("cartographer_example", "cartographer.launch",
            arguments=f"config_filename:={config_filename} load_state_filename:={load_state_filename}",
            source_files=self.source_files)
        return result
