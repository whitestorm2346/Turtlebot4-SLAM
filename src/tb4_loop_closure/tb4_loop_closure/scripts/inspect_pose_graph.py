#!/usr/bin/env python3

"""Inspect raw and optimized RTAB-Map pose graphs."""

import rclpy
import numpy as np
from rclpy.node import Node

from tb4_loop_closure.graph.pose_graph_snapshot import (
    PoseGraph,
    PoseGraphSnapshot,
)

from tb4_loop_closure.verification.trajectory_prior_constraint import (
    TrajectoryPriorConstraint,
)

import inspect

import tb4_loop_closure.verification.trajectory_prior_constraint as tpc_module


class PoseGraphInspector(Node):
    """CLI utility for inspecting RTAB-Map pose graphs."""

    def __init__(self) -> None:
        super().__init__("pose_graph_inspector")

        self.snapshot = PoseGraphSnapshot(
            node=self,
            service_name="/rtabmap/get_map_data",
        )

    @staticmethod
    def print_graph_summary(
        graph: PoseGraph,
        max_nodes: int = 10,
    ) -> None:
        """Print one graph summary."""
        graph_name = (
            "Optimized Pose Graph"
            if graph.optimized
            else "Raw Pose Graph"
        )

        print()
        print("=" * 70)
        print(graph_name)
        print("=" * 70)
        print(f"Number of Nodes : {graph.node_count}")
        print(f"Number of Links : {graph.link_count}")

        print()
        print(f"First {min(max_nodes, graph.node_count)} Nodes")
        print("-" * 70)
        print(f"{'ID':>8} {'X':>12} {'Y':>12} {'Z':>12}")

        samples = sorted(
            zip(graph.node_ids, graph.poses),
            key=lambda item: item[0],
        )

        for node_id, pose in samples[:max_nodes]:
            print(
                f"{node_id:8d}"
                f"{pose.position.x:12.3f}"
                f"{pose.position.y:12.3f}"
                f"{pose.position.z:12.3f}"
            )

        grouped_links = graph.links_by_type()

        print()
        print("Links by Type")
        print("-" * 70)

        for link_type in sorted(grouped_links):
            print(
                f"type={link_type:<3d} "
                f"count={len(grouped_links[link_type])}"
            )

        # In the current RTAB-Map output:
        # type 0 normally represents neighboring/odometry constraints.
        # type 1 normally represents global loop-closure constraints.
        loop_links = grouped_links.get(1, [])

        print()
        print(f"First {min(10, len(loop_links))} Type-1 Links")
        print("-" * 70)

        for link in loop_links[:10]:
            print(
                f"{link.from_id:8d}"
                f" -> "
                f"{link.to_id:8d}"
            )

    def run(self) -> None:
        print()
        print("=" * 70)
        print("Loaded Python Modules")
        print("=" * 70)
        print(f"TPC module : {tpc_module.__file__}")
        print(
            "TPC class  : "
            f"{inspect.getfile(TrajectoryPriorConstraint)}"
        )

        """Retrieve and compare raw and optimized graphs."""
        if not self.snapshot.wait_for_service(timeout_sec=10.0):
            return

        self.get_logger().info("Requesting raw pose graph...")

        raw_graph = self.snapshot.get_graph(
            optimized=False,
            global_map=True,
            graph_only=True,
        )

        self.get_logger().info("Requesting optimized pose graph...")

        optimized_graph = self.snapshot.get_graph(
            optimized=True,
            global_map=True,
            graph_only=True,
        )

        self.print_graph_summary(raw_graph)
        self.print_graph_summary(optimized_graph)

        (
            common_ids,
            raw_trajectory,
            optimized_trajectory,
        ) = self.snapshot.align_common_nodes(
            raw_graph,
            optimized_graph,
            dimensions=3,
        )

        print()
        print("=" * 70)
        print("Trajectory Pair")
        print("=" * 70)
        print(f"Common Nodes              : {len(common_ids)}")
        print(f"Raw Trajectory Shape      : {raw_trajectory.shape}")
        print(
            "Optimized Trajectory Shape: "
            f"{optimized_trajectory.shape}"
        )

        print()
        print("First 10 Matching Poses")
        print("-" * 70)
        print(
            f"{'ID':>8} "
            f"{'Raw X':>10} {'Raw Y':>10} "
            f"{'Opt X':>10} {'Opt Y':>10}"
        )

        for index in range(min(10, len(common_ids))):
            print(
                f"{common_ids[index]:8d} "
                f"{raw_trajectory[index, 0]:10.3f} "
                f"{raw_trajectory[index, 1]:10.3f} "
                f"{optimized_trajectory[index, 0]:10.3f} "
                f"{optimized_trajectory[index, 1]:10.3f}"
            )

        # ==========================================================
        # Trajectory Prior Constraint
        # ==========================================================

        print()
        print("=" * 70)
        print("Trajectory Prior Constraint")
        print("=" * 70)

        # 目前只是 Integration Test
        # threshold 不具決策意義，因此設很大
        tpc = TrajectoryPriorConstraint(
            threshold=float("inf")
        )

        result = tpc.compute_score(
            raw_trajectory,
            optimized_trajectory,
        )

        aligned = result["aligned_trajectory"]

        recomputed_difference = raw_trajectory - aligned

        recomputed_score = float(
            np.sqrt(
                np.mean(
                    np.sum(
                        recomputed_difference ** 2,
                        axis=1,
                    )
                )
            )
        )

        print()
        print("TPC Consistency Check")
        print("-" * 70)
        print(f"Returned Score   : {result['score']:.12f}")
        print(f"Recomputed Score : {recomputed_score:.12f}")
        print(
            "Score Difference : "
            f"{abs(result['score'] - recomputed_score):.12f}"
        )

        if not np.isclose(
            result["score"],
            recomputed_score,
            rtol=1e-9,
            atol=1e-12,
        ):
            raise RuntimeError(
                "TPC score is inconsistent with aligned_trajectory. "
                "Check which trajectory_prior_constraint.py is being imported."
            )

        # ----------------------------------------------------------
        # Alignment improvement
        # ----------------------------------------------------------

        before_rmse = np.sqrt(
            np.mean(
                np.sum(
                    (raw_trajectory - optimized_trajectory) ** 2,
                    axis=1,
                )
            )
        )

        after_rmse = np.sqrt(
            np.mean(
                np.sum(
                    (raw_trajectory - aligned) ** 2,
                    axis=1,
                )
            )
        )

        print()
        print("Alignment Improvement")
        print("-" * 70)

        print(f"Before Alignment RMSE : {before_rmse:.6f}")
        print(f"After  Alignment RMSE : {after_rmse:.6f}")

        # ----------------------------------------------------------
        # TPC Result
        # ----------------------------------------------------------

        print()
        print("TPC Result")
        print("-" * 70)

        print(f"Compared Nodes : {len(common_ids)}")
        print(f"TPC Score      : {result['score']:.6f}")
        print(f"Passed         : {result['passed']}")

        print()
        print("Rotation")

        print(result["rotation"])

        print()
        print("Translation")

        print(result["translation"])

        # ----------------------------------------------------------
        # Deformation Statistics
        # ----------------------------------------------------------

        deformation = np.linalg.norm(
            raw_trajectory - aligned,
            axis=1,
        )

        print()
        print("Deformation Statistics")
        print("-" * 70)

        print(f"Mean : {deformation.mean():.6f}")
        print(f"Std  : {deformation.std():.6f}")
        print(f"Max  : {deformation.max():.6f}")
        print(f"Min  : {deformation.min():.6f}")

        # ----------------------------------------------------------
        # Largest deformation
        # ----------------------------------------------------------

        largest = np.argsort(deformation)[::-1]

        print()
        print("Top 10 Deformed Nodes")
        print("-" * 70)

        print(f"{'Node':>8} {'Error(m)':>12}")

        for idx in largest[:10]:

            print(
                f"{common_ids[idx]:8d}"
                f"{deformation[idx]:12.6f}"
            )


def main(args=None) -> None:
    """Run the pose graph inspector."""
    rclpy.init(args=args)

    np.set_printoptions(
        precision=4,
        suppress=True,
    )

    node = PoseGraphInspector()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    except Exception as error:
        node.get_logger().error(
            f"Pose graph inspection failed: {error}"
        )
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()