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

    def create_containter(self, mounts: CartographerMounts=None, net='host'):
        if mounts is None:
            mounts = CartographerMounts()
        super().create_containter(mounts=mounts, net=net)

        self.run(f"echo 'export artd_COL_LOG_FOLDER=/home/docker_cartographer/catkin_ws/cartographer_logs' >> ~/.bashrc", quiet=True)
        self.run(f"echo 'export displacement_COL_LOG_FOLDER=/home/docker_cartographer/catkin_ws/cartographer_logs' >> ~/.bashrc", quiet=True)
        self.run(f"echo 'export trim_loops_COL_LOG_FOLDER=/home/docker_cartographer/catkin_ws/cartographer_logs' >> ~/.bashrc", quiet=True)

    def build_cartographer(self):
        result = self.run("cd ~/catkin_ws && catkin_make_isolated -DCMAKE_CXX_STANDARD=17 --use-ninja")
        return result

    def run_cartographer(self, config_filename,
            load_state_filename=None, save_state_filename=None):
        if load_state_filename is None:
            load_state_filename = '""'
        if save_state_filename is None:
            save_state_filename = '""'
        result = self.roslaunch("cartographer_example", "cartographer.launch",
            arguments=
                f"config_filename:={config_filename} "
                f"load_state_filename:={load_state_filename} "
                f"save_state_filename:={save_state_filename} ",
            source_files=self.source_files)
        return result
