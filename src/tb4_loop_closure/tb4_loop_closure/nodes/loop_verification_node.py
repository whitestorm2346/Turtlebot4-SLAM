#!/usr/bin/env python3

"""
ROS 2 node for online RTAB-Map loop-closure monitoring.

Current responsibilities:
    1. Periodically request the optimized RTAB-Map pose graph.
    2. Detect newly added type-1 links.
    3. Print each newly detected loop closure once.

TPC verification will be connected after the loop-monitoring stage is
confirmed to work reliably.
"""

from __future__ import annotations

import traceback
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from tb4_loop_closure.graph.pose_graph_snapshot import (
    PoseGraph,
    PoseGraphSnapshot,
)
from tb4_loop_closure.monitoring.loop_closure_monitor import (
    LoopClosureEvent,
    LoopClosureMonitor,
)


class LoopVerificationNode(Node):
    """Monitor RTAB-Map for newly added loop closures."""

    def __init__(self) -> None:
        super().__init__("loop_verification_node")

        self.declare_parameter("check_period", 1.0)
        self.declare_parameter("use_optimized_graph", True)
        self.declare_parameter("print_no_change", False)

        self._check_period = float(
            self.get_parameter("check_period").value
        )

        self._use_optimized_graph = bool(
            self.get_parameter("use_optimized_graph").value
        )

        self._print_no_change = bool(
            self.get_parameter("print_no_change").value
        )

        if self._check_period <= 0.0:
            raise ValueError(
                "Parameter 'check_period' must be greater than zero."
            )

        self._snapshot = PoseGraphSnapshot(self)
        self._monitor = LoopClosureMonitor()

        # Stores the service Future while a graph request is running.
        self._pending_future: Optional[Future] = None

        # Avoid printing the same service warning every second.
        self._service_warning_printed = False

        self._timer = self.create_timer(
            self._check_period,
            self._request_pose_graph,
        )

        self.get_logger().info(
            "LoopVerificationNode started."
        )

        self.get_logger().info(
            f"Check period        : "
            f"{self._check_period:.2f} seconds"
        )

        self.get_logger().info(
            "Graph mode          : "
            f"{'optimized' if self._use_optimized_graph else 'raw'}"
        )

        self.get_logger().info(
            "Waiting for the first RTAB-Map pose graph..."
        )

    def _request_pose_graph(self) -> None:
        """
        Timer callback that starts a non-blocking graph request.
        """
        # Do not issue another request before the previous one finishes.
        if self._pending_future is not None:
            if not self._pending_future.done():
                return

            # The completed request should normally have been cleared by
            # _on_pose_graph_response(). This is only a defensive fallback.
            self._pending_future = None

        if not self._snapshot.service_is_ready():
            if not self._service_warning_printed:
                self.get_logger().warning(
                    f"Waiting for service "
                    f"{self._snapshot.service_name} ..."
                )
                self._service_warning_printed = True

            return

        if self._service_warning_printed:
            self.get_logger().info(
                f"Service {self._snapshot.service_name} is available."
            )
            self._service_warning_printed = False

        try:
            future = self._snapshot.get_graph_async(
                optimized=self._use_optimized_graph,
                global_map=True,
                graph_only=True,
            )

            self._pending_future = future

            future.add_done_callback(
                self._on_pose_graph_response
            )

        except Exception as error:
            self._pending_future = None

            self.get_logger().error(
                "Failed to start pose-graph request: "
                f"{type(error).__name__}: {error}"
            )

            self.get_logger().debug(
                traceback.format_exc()
            )

    def _on_pose_graph_response(
        self,
        future: Future,
    ) -> None:
        """
        Process a completed RTAB-Map service request.
        """
        try:
            graph = self._snapshot.graph_from_future(
                future=future,
                optimized=self._use_optimized_graph,
            )

            self._process_pose_graph(graph)

        except Exception as error:
            self.get_logger().error(
                "Failed to process the RTAB-Map pose graph: "
                f"{type(error).__name__}: {error}"
            )

            self.get_logger().debug(
                traceback.format_exc()
            )

        finally:
            if self._pending_future is future:
                self._pending_future = None

    def _process_pose_graph(
        self,
        graph: PoseGraph,
    ) -> None:
        """
        Pass the newest pose graph to LoopClosureMonitor.
        """
        was_initialized = self._monitor.initialized

        new_events = self._monitor.detect_new_links(graph)

        # The first graph is only used to establish the baseline.
        if not was_initialized and self._monitor.initialized:
            self.get_logger().info(
                "Loop-closure monitor initialized."
            )

            self.get_logger().info(
                f"Graph nodes         : {graph.node_count}"
            )

            self.get_logger().info(
                f"Graph links         : {graph.link_count}"
            )

            self.get_logger().info(
                "Existing loop links : "
                f"{self._monitor.known_loop_count}"
            )

            return

        if not new_events:
            if self._print_no_change:
                self.get_logger().info(
                    "No new loop closure detected. "
                    f"Nodes={graph.node_count}, "
                    f"links={graph.link_count}, "
                    f"known_loops={self._monitor.known_loop_count}"
                )

            return

        for event in new_events:
            self._handle_loop_closure_event(
                event=event,
                graph=graph,
            )

    def _handle_loop_closure_event(
        self,
        event: LoopClosureEvent,
        graph: PoseGraph,
    ) -> None:
        """
        Handle one newly detected loop closure.

        Global TPC verification will later be connected here.
        """
        self.get_logger().info(
            "\n"
            "========================================\n"
            "New Loop Closure Detected\n"
            f"From node  : {event.from_id}\n"
            f"To node    : {event.to_id}\n"
            f"Link type  : {event.link_type}\n"
            f"Graph nodes: {graph.node_count}\n"
            f"Graph links: {graph.link_count}\n"
            "========================================"
        )


def main(args: list[str] | None = None) -> None:
    """ROS 2 entry point."""
    rclpy.init(args=args)

    node: LoopVerificationNode | None = None

    try:
        node = LoopVerificationNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if node is not None:
            node.get_logger().fatal(
                "LoopVerificationNode terminated unexpectedly: "
                f"{type(error).__name__}: {error}"
            )
        else:
            print(
                "LoopVerificationNode failed during initialization: "
                f"{type(error).__name__}: {error}"
            )

        raise

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()