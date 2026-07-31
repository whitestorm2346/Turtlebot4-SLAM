#!/usr/bin/env python3

"""
Online RTAB-Map loop-closure verification node.

Pipeline
--------
1. Periodically request the optimized RTAB-Map pose graph.
2. Detect newly added type-1 loop-closure links.
3. When a new loop closure is detected, request the raw pose graph.
4. Match raw and optimized trajectories using common node IDs.
5. Compute a global Trajectory Prior Constraint (TPC) score.
6. Print the verification result.

Notes
-----
This version computes a GLOBAL TPC score using all common nodes in the
raw and optimized pose graphs.

Local trajectory extraction will be added later.
"""

from __future__ import annotations

import traceback
from enum import Enum
from typing import List, Optional

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
from tb4_loop_closure.verification.trajectory_prior_constraint import (
    TrajectoryPriorConstraint,
)


class RequestState(Enum):
    """Current service-request state."""

    IDLE = "idle"
    WAITING_OPTIMIZED = "waiting_optimized"
    WAITING_RAW = "waiting_raw"


class LoopVerificationNode(Node):
    """
    Detect and verify newly added RTAB-Map loop closures.

    This node uses a non-blocking service-call state machine:

        IDLE
          ↓
        request optimized graph
          ↓
        WAITING_OPTIMIZED
          ↓
        detect new loop closures
          ↓
        request raw graph
          ↓
        WAITING_RAW
          ↓
        compute global TPC
          ↓
        IDLE
    """

    def __init__(self) -> None:
        super().__init__("loop_verification_node")

        # -------------------------------------------------------------
        # ROS parameters
        # -------------------------------------------------------------

        self.declare_parameter("check_period", 1.0)
        self.declare_parameter("print_no_change", False)
        self.declare_parameter("trajectory_dimensions", 3)

        self._check_period = float(
            self.get_parameter("check_period").value
        )

        self._print_no_change = bool(
            self.get_parameter("print_no_change").value
        )

        self._trajectory_dimensions = int(
            self.get_parameter("trajectory_dimensions").value
        )

        if self._check_period <= 0.0:
            raise ValueError(
                "Parameter 'check_period' must be greater than zero."
            )

        if self._trajectory_dimensions not in (2, 3):
            raise ValueError(
                "Parameter 'trajectory_dimensions' must be 2 or 3."
            )

        # -------------------------------------------------------------
        # Pipeline components
        # -------------------------------------------------------------

        self._snapshot = PoseGraphSnapshot(self)
        self._monitor = LoopClosureMonitor()

        # Uses the threshold configured by the existing
        # TrajectoryPriorConstraint implementation.
        self._tpc = TrajectoryPriorConstraint(
            threshold=float("inf")
        )

        # -------------------------------------------------------------
        # Asynchronous request state
        # -------------------------------------------------------------

        self._state = RequestState.IDLE

        self._pending_future: Optional[Future] = None

        # Optimized graph associated with the newly detected loop closure.
        self._pending_optimized_graph: Optional[PoseGraph] = None

        # New loop closures found in that optimized graph.
        self._pending_events: List[LoopClosureEvent] = []

        # Avoid printing the same service warning every timer cycle.
        self._service_warning_printed = False

        # Prevent stale callbacks from processing an old request.
        self._request_sequence = 0

        # -------------------------------------------------------------
        # Timer
        # -------------------------------------------------------------

        self._timer = self.create_timer(
            self._check_period,
            self._on_timer,
        )

        self.get_logger().info(
            "LoopVerificationNode started."
        )

        self.get_logger().info(
            f"Check period          : "
            f"{self._check_period:.2f} seconds"
        )

        self.get_logger().info(
            f"Trajectory dimensions : "
            f"{self._trajectory_dimensions}D"
        )

        self.get_logger().info(
            "Verification mode     : Global TPC"
        )

        self.get_logger().info(
            "Waiting for the first RTAB-Map pose graph..."
        )

    # -----------------------------------------------------------------
    # Timer and request control
    # -----------------------------------------------------------------

    def _on_timer(self) -> None:
        """
        Periodically start an optimized graph request.

        While a previous optimized/raw request is still running, this timer
        callback does nothing.
        """
        if self._state is not RequestState.IDLE:
            return

        if not self._snapshot.service_is_ready():
            self._print_service_waiting_warning()
            return

        self._print_service_available_message()

        self._start_optimized_request()

    def _print_service_waiting_warning(self) -> None:
        """Print the service warning only once per unavailable period."""
        if self._service_warning_printed:
            return

        self.get_logger().warning(
            f"Waiting for service "
            f"{self._snapshot.service_name} ..."
        )

        self._service_warning_printed = True

    def _print_service_available_message(self) -> None:
        """Print a recovery message after the service becomes available."""
        if not self._service_warning_printed:
            return

        self.get_logger().info(
            f"Service {self._snapshot.service_name} is available."
        )

        self._service_warning_printed = False

    def _start_optimized_request(self) -> None:
        """Start a non-blocking optimized pose-graph request."""
        try:
            self._state = RequestState.WAITING_OPTIMIZED
            self._request_sequence += 1

            request_sequence = self._request_sequence

            future = self._snapshot.get_graph_async(
                optimized=True,
                global_map=True,
                graph_only=True,
            )

            self._pending_future = future

            future.add_done_callback(
                lambda completed_future: self._on_optimized_response(
                    completed_future,
                    request_sequence,
                )
            )

        except Exception as error:
            self._reset_pipeline_state()

            self.get_logger().error(
                "Failed to start optimized graph request: "
                f"{type(error).__name__}: {error}"
            )

            self.get_logger().debug(
                traceback.format_exc()
            )

    def _start_raw_request(self) -> None:
        """Start a non-blocking raw pose-graph request."""
        try:
            self._state = RequestState.WAITING_RAW
            self._request_sequence += 1

            request_sequence = self._request_sequence

            future = self._snapshot.get_graph_async(
                optimized=False,
                global_map=True,
                graph_only=True,
            )

            self._pending_future = future

            future.add_done_callback(
                lambda completed_future: self._on_raw_response(
                    completed_future,
                    request_sequence,
                )
            )

        except Exception as error:
            self.get_logger().error(
                "Failed to start raw graph request: "
                f"{type(error).__name__}: {error}"
            )

            self.get_logger().debug(
                traceback.format_exc()
            )

            self._reset_pipeline_state()

    # -----------------------------------------------------------------
    # Optimized graph processing
    # -----------------------------------------------------------------

    def _on_optimized_response(
        self,
        future: Future,
        request_sequence: int,
    ) -> None:
        """Process a completed optimized-graph request."""
        if request_sequence != self._request_sequence:
            self.get_logger().warning(
                "Ignoring stale optimized graph response."
            )
            return

        if self._state is not RequestState.WAITING_OPTIMIZED:
            self.get_logger().warning(
                "Received an optimized graph response in an "
                f"unexpected state: {self._state.value}"
            )
            return

        try:
            optimized_graph = self._snapshot.graph_from_future(
                future=future,
                optimized=True,
            )

            self._pending_future = None

            self._process_optimized_graph(
                optimized_graph
            )

        except Exception as error:
            self.get_logger().error(
                "Failed to process optimized pose graph: "
                f"{type(error).__name__}: {error}"
            )

            self.get_logger().debug(
                traceback.format_exc()
            )

            self._reset_pipeline_state()

    def _process_optimized_graph(
        self,
        optimized_graph: PoseGraph,
    ) -> None:
        """
        Detect new loop closures from the optimized graph.

        When one or more new loop closures are found, retain this optimized
        graph and request the corresponding raw graph.
        """
        was_initialized = self._monitor.initialized

        new_events = self._monitor.detect_new_links(
            optimized_graph
        )

        # First graph establishes the monitor baseline.
        if not was_initialized and self._monitor.initialized:
            self.get_logger().info(
                "Loop-closure monitor initialized."
            )

            self.get_logger().info(
                f"Graph nodes           : "
                f"{optimized_graph.node_count}"
            )

            self.get_logger().info(
                f"Graph links           : "
                f"{optimized_graph.link_count}"
            )

            self.get_logger().info(
                f"Existing loop links   : "
                f"{self._monitor.known_loop_count}"
            )

            self._reset_pipeline_state()
            return

        if not new_events:
            if self._print_no_change:
                loop_count = self._count_loop_closures(
                    optimized_graph
                )

                self.get_logger().info(
                    "No new loop closure detected. "
                    f"nodes={optimized_graph.node_count}, "
                    f"links={optimized_graph.link_count}, "
                    f"loops={loop_count}"
                )

            self._reset_pipeline_state()
            return

        # Keep the optimized graph fixed while its corresponding raw graph
        # is requested.
        self._pending_optimized_graph = optimized_graph
        self._pending_events = list(new_events)

        for event in new_events:
            self.get_logger().info(
                "New loop closure detected: "
                f"{event.from_id} -> {event.to_id}"
            )

        self.get_logger().info(
            "Requesting raw pose graph for Global TPC..."
        )

        self._start_raw_request()

    # -----------------------------------------------------------------
    # Raw graph and TPC processing
    # -----------------------------------------------------------------

    def _on_raw_response(
        self,
        future: Future,
        request_sequence: int,
    ) -> None:
        """Process a completed raw-graph request."""
        if request_sequence != self._request_sequence:
            self.get_logger().warning(
                "Ignoring stale raw graph response."
            )
            return

        if self._state is not RequestState.WAITING_RAW:
            self.get_logger().warning(
                "Received a raw graph response in an unexpected state: "
                f"{self._state.value}"
            )
            return

        try:
            raw_graph = self._snapshot.graph_from_future(
                future=future,
                optimized=False,
            )

            self._pending_future = None

            optimized_graph = self._pending_optimized_graph

            if optimized_graph is None:
                raise RuntimeError(
                    "The optimized graph associated with this raw "
                    "graph request is missing."
                )

            if not self._pending_events:
                raise RuntimeError(
                    "No pending loop-closure events are available."
                )

            self._compute_global_tpc(
                raw_graph=raw_graph,
                optimized_graph=optimized_graph,
                events=self._pending_events,
            )

        except Exception as error:
            self.get_logger().error(
                "Failed to compute Global TPC: "
                f"{type(error).__name__}: {error}"
            )

            self.get_logger().debug(
                traceback.format_exc()
            )

        finally:
            self._reset_pipeline_state()

    def _compute_global_tpc(
        self,
        raw_graph: PoseGraph,
        optimized_graph: PoseGraph,
        events: List[LoopClosureEvent],
    ) -> None:
        """
        Compute the Global TPC score for newly detected loop closures.

        The raw and optimized trajectories are paired using their common
        node IDs before being passed to TrajectoryPriorConstraint.
        """
        (
            common_ids,
            raw_trajectory,
            optimized_trajectory,
        ) = PoseGraphSnapshot.align_common_nodes(
            first=raw_graph,
            second=optimized_graph,
            dimensions=self._trajectory_dimensions,
        )

        if len(common_ids) < 3:
            raise ValueError(
                "At least three common graph nodes are required for "
                f"trajectory alignment, but only {len(common_ids)} "
                "were found."
            )

        result = self._tpc.compute_score(
            raw_trajectory,
            optimized_trajectory,
        )

        if not isinstance(result, dict):
            raise TypeError(
                "TrajectoryPriorConstraint.compute_score() must "
                "return a dictionary."
            )

        if "score" not in result:
            raise KeyError(
                "TPC result does not contain the 'score' field."
            )

        if "passed" not in result:
            raise KeyError(
                "TPC result does not contain the 'passed' field."
            )

        score = float(result["score"])
        passed = bool(result["passed"])

        loop_count = self._count_loop_closures(
            optimized_graph
        )

        # All events detected in this optimized graph share the same
        # global raw-vs-optimized trajectory comparison.
        for event in events:
            self._log_tpc_result(
                event=event,
                score=score,
                passed=passed,
                common_node_count=len(common_ids),
                raw_graph=raw_graph,
                optimized_graph=optimized_graph,
                loop_count=loop_count,
            )

    def _log_tpc_result(
        self,
        event: LoopClosureEvent,
        score: float,
        passed: bool,
        common_node_count: int,
        raw_graph: PoseGraph,
        optimized_graph: PoseGraph,
        loop_count: int,
    ) -> None:
        """Print one Global TPC verification result."""
        decision = "PASS" if passed else "FAIL"

        message = (
            "\n"
            "==================================================\n"
            "Global Loop Closure Verification\n"
            f"Loop closure          : "
            f"{event.from_id} -> {event.to_id}\n"
            f"Link type             : {event.link_type}\n"
            f"TPC score             : {score:.6f} m\n"
            f"Decision              : {decision}\n"
            f"Common trajectory nodes: {common_node_count}\n"
            f"Raw graph nodes       : {raw_graph.node_count}\n"
            f"Optimized graph nodes : {optimized_graph.node_count}\n"
            f"Optimized graph links : {optimized_graph.link_count}\n"
            f"Total loop closures   : {loop_count}\n"
            "=================================================="
        )

        if passed:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)

    # -----------------------------------------------------------------
    # Utility methods
    # -----------------------------------------------------------------

    @staticmethod
    def _count_loop_closures(
        graph: PoseGraph,
    ) -> int:
        """Count type-1 links in a pose graph."""
        grouped_links = graph.links_by_type()

        return len(
            grouped_links.get(
                LoopClosureMonitor.LOOP_CLOSURE_TYPE,
                [],
            )
        )

    def _reset_pipeline_state(self) -> None:
        """Return the asynchronous pipeline to its idle state."""
        self._pending_future = None
        self._pending_optimized_graph = None
        self._pending_events = []
        self._state = RequestState.IDLE


def main(args: list[str] | None = None) -> None:
    """ROS 2 executable entry point."""
    rclpy.init(args=args)

    node: Optional[LoopVerificationNode] = None

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