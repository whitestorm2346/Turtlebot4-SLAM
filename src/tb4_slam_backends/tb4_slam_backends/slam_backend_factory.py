from tb4_slam_backends.rtabmap_backend import RTABMapBackend
from tb4_slam_backends.dense_slam_backend import DenseSlamBackend
from tb4_slam_backends.orb_slam_backend import ORBSlamBackend


def create_slam_backend(backend_name, node):
    if backend_name == 'rtabmap':
        return RTABMapBackend(node)

    if backend_name == 'dense_slam':
        return DenseSlamBackend(node)

    if backend_name == 'orb_slam':
        return ORBSlamBackend(node)

    raise ValueError(f'Unknown SLAM backend: {backend_name}')