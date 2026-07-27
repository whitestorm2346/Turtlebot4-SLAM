"""Utilities for retrieving RTAB-Map pose graph snapshots."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.task import Future
from rtabmap_msgs.msg import Link
from rtabmap_msgs.srv import GetMap


@dataclass
class PoseGraph:
    """A simplified RTAB-Map pose graph."""

    node_ids: List[int]
    poses: List[Pose]
    links: List[Link]
    optimized: bool

    def __post_init__(self) -> None:
        """Validate graph fields."""
        if len(self.node_ids) != len(self.poses):
            raise ValueError(
                "node_ids and poses must have the same length: "
                f"{len(self.node_ids)} != {len(self.poses)}"
            )

    @property
    def node_count(self) -> int:
        """Return the number of graph nodes."""
        return len(self.node_ids)

    @property
    def link_count(self) -> int:
        """Return the number of graph links."""
        return len(self.links)

    def pose_dict(self) -> Dict[int, Pose]:
        """Return poses indexed by node ID."""
        return {
            int(node_id): pose
            for node_id, pose in zip(self.node_ids, self.poses)
        }

    def to_numpy(
        self,
        dimensions: int = 3,
        sort_by_id: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert graph poses into NumPy arrays.

        Parameters
        ----------
        dimensions:
            Use 2 for [x, y], or 3 for [x, y, z].
        sort_by_id:
            Sort trajectory samples by node ID.

        Returns
        -------
        node_ids:
            Shape (N,), containing graph node IDs.
        trajectory:
            Shape (N, 2) or (N, 3), containing positions.
        """
        if dimensions not in (2, 3):
            raise ValueError("dimensions must be either 2 or 3")

        samples = list(zip(self.node_ids, self.poses))

        if sort_by_id:
            samples.sort(key=lambda item: item[0])

        node_ids = np.asarray(
            [node_id for node_id, _ in samples],
            dtype=np.int64,
        )

        if dimensions == 2:
            trajectory = np.asarray(
                [
                    [
                        pose.position.x,
                        pose.position.y,
                    ]
                    for _, pose in samples
                ],
                dtype=np.float64,
            )
        else:
            trajectory = np.asarray(
                [
                    [
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    ]
                    for _, pose in samples
                ],
                dtype=np.float64,
            )

        return node_ids, trajectory

    def links_by_type(self) -> Dict[int, List[Link]]:
        """Group graph links using the RTAB-Map link type integer."""
        grouped: Dict[int, List[Link]] = {}

        for link in self.links:
            link_type = int(link.type)
            grouped.setdefault(link_type, []).append(link)

        return grouped


class PoseGraphSnapshot:
    """
    Retrieve pose graph snapshots from RTAB-Map.

    This class provides two request styles:

    get_graph():
        Synchronous request intended for one-shot scripts that are not
        already running inside rclpy.spin().

    get_graph_async():
        Asynchronous request intended for ROS 2 nodes that are already
        being handled by an executor.
    """

    def __init__(
        self,
        node: Node,
        service_name: str = "/rtabmap/get_map_data",
    ) -> None:
        self._node = node
        self._service_name = service_name

        self._client = node.create_client(
            GetMap,
            service_name,
        )

    @property
    def service_name(self) -> str:
        """Return the RTAB-Map service name."""
        return self._service_name

    def service_is_ready(self) -> bool:
        """Return whether the RTAB-Map service is currently available."""
        return self._client.service_is_ready()

    def wait_for_service(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait until the RTAB-Map service becomes available.

        Returns True when available, otherwise False after timeout.
        """
        self._node.get_logger().info(
            f"Waiting for {self._service_name} ..."
        )

        available = self._client.wait_for_service(
            timeout_sec=timeout_sec,
        )

        if not available:
            self._node.get_logger().error(
                f"Service {self._service_name} is not available."
            )

        return available

    @staticmethod
    def _create_request(
        optimized: bool,
        global_map: bool,
        graph_only: bool,
    ) -> GetMap.Request:
        """Create an RTAB-Map GetMap service request."""
        request = GetMap.Request()
        request.global_map = bool(global_map)
        request.optimized = bool(optimized)
        request.graph_only = bool(graph_only)

        return request

    def get_graph_async(
        self,
        optimized: bool,
        global_map: bool = True,
        graph_only: bool = True,
    ) -> Future:
        """
        Start an asynchronous RTAB-Map graph request.

        This method does not spin and does not block.

        Parameters
        ----------
        optimized:
            True requests optimized poses.
            False requests non-optimized poses.
        global_map:
            True requests the complete graph.
        graph_only:
            True avoids requesting image and sensor payloads.

        Returns
        -------
        Future
            The ROS 2 service future. Pass the completed future to
            graph_from_future().
        """
        if not self._client.service_is_ready():
            raise RuntimeError(
                f"Service {self._service_name} is not available."
            )

        request = self._create_request(
            optimized=optimized,
            global_map=global_map,
            graph_only=graph_only,
        )

        future = self._client.call_async(request)

        # Save the requested graph mode on the Future so that the response
        # can later be converted into the correct PoseGraph object.
        setattr(
            future,
            "_pose_graph_optimized",
            bool(optimized),
        )

        return future

    def graph_from_future(
        self,
        future: Future,
        optimized: bool | None = None,
    ) -> PoseGraph:
        """
        Convert a completed GetMap service Future into a PoseGraph.

        Parameters
        ----------
        future:
            A completed Future returned by get_graph_async().
        optimized:
            Optional explicit graph mode. Usually this can be omitted because
            get_graph_async() stores the requested mode on the Future.
        """
        if future is None:
            raise ValueError("future cannot be None")

        if not future.done():
            raise RuntimeError(
                "The RTAB-Map service future is not complete."
            )

        if future.cancelled():
            raise RuntimeError(
                "The RTAB-Map service request was cancelled."
            )

        exception = future.exception()

        if exception is not None:
            raise RuntimeError(
                f"RTAB-Map service call failed: {exception}"
            ) from exception

        response = future.result()

        if response is None:
            raise RuntimeError(
                "RTAB-Map service returned no response."
            )

        if optimized is None:
            optimized = bool(
                getattr(
                    future,
                    "_pose_graph_optimized",
                    False,
                )
            )

        return self._response_to_pose_graph(
            response=response,
            optimized=optimized,
        )

    @staticmethod
    def _response_to_pose_graph(
        response: GetMap.Response,
        optimized: bool,
    ) -> PoseGraph:
        """Convert an RTAB-Map GetMap response into PoseGraph."""
        graph = response.data.graph

        return PoseGraph(
            node_ids=[
                int(node_id)
                for node_id in graph.poses_id
            ],
            poses=list(graph.poses),
            links=list(graph.links),
            optimized=bool(optimized),
        )

    def get_graph(
        self,
        optimized: bool,
        global_map: bool = True,
        graph_only: bool = True,
        timeout_sec: float = 30.0,
    ) -> PoseGraph:
        """
        Request one RTAB-Map pose graph synchronously.

        Important
        ---------
        Use this method only in one-shot scripts where the node is not already
        running inside rclpy.spin().

        For continuously running ROS 2 nodes, use get_graph_async() instead.
        """
        if not self._client.service_is_ready():
            available = self._client.wait_for_service(
                timeout_sec=timeout_sec,
            )

            if not available:
                raise TimeoutError(
                    f"Service {self._service_name} was not available "
                    f"within {timeout_sec:.1f} seconds."
                )

        future = self.get_graph_async(
            optimized=optimized,
            global_map=global_map,
            graph_only=graph_only,
        )

        rclpy.spin_until_future_complete(
            self._node,
            future,
            timeout_sec=timeout_sec,
        )

        if not future.done():
            future.cancel()

            raise TimeoutError(
                f"Timed out while calling {self._service_name}"
            )

        return self.graph_from_future(
            future=future,
            optimized=optimized,
        )

    @staticmethod
    def align_common_nodes(
        first: PoseGraph,
        second: PoseGraph,
        dimensions: int = 3,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Extract matching nodes from two graphs.

        Returns
        -------
        common_ids:
            Shape (N,)
        first_trajectory:
            Shape (N, dimensions)
        second_trajectory:
            Shape (N, dimensions)
        """
        if dimensions not in (2, 3):
            raise ValueError("dimensions must be either 2 or 3")

        first_poses = first.pose_dict()
        second_poses = second.pose_dict()

        common_ids = sorted(
            set(first_poses.keys()) & set(second_poses.keys())
        )

        if not common_ids:
            raise ValueError(
                "The two pose graphs do not share any node IDs."
            )

        def pose_to_array(pose: Pose) -> List[float]:
            values = [
                float(pose.position.x),
                float(pose.position.y),
            ]

            if dimensions == 3:
                values.append(float(pose.position.z))

            return values

        first_trajectory = np.asarray(
            [
                pose_to_array(first_poses[node_id])
                for node_id in common_ids
            ],
            dtype=np.float64,
        )

        second_trajectory = np.asarray(
            [
                pose_to_array(second_poses[node_id])
                for node_id in common_ids
            ],
            dtype=np.float64,
        )

        return (
            np.asarray(common_ids, dtype=np.int64),
            first_trajectory,
            second_trajectory,
        )