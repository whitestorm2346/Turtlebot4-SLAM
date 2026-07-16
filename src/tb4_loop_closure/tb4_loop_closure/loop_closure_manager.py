import rclpy
from rclpy.node import Node

from tb4_loop_closure.rtabmap_candidate_monitor import (
    RTABMapCandidateMonitor
)

from tb4_loop_closure.verifiers.temporal_verifier import (
    TemporalVerifier
)

from tb4_loop_closure.verifiers.graph_verifier import (
    GraphConsistencyVerifier
)

class LoopClosureManager(Node):

    def __init__(self):
        super().__init__('loop_closure_manager')

        self.candidate_monitor = RTABMapCandidateMonitor(
            node=self,
            candidate_callback=self.handle_candidate
        )

        self.temporal_verifier = TemporalVerifier(
            window_size=5,
            min_consistent_ratio=0.6,
            max_matched_stagnation=2
        )

        self.graph_verifier = GraphConsistencyVerifier(
            warning_error_threshold=20.0,
            reject_error_threshold=100.0,
            max_error_ratio_threshold=5.0,
            max_ang_error_ratio_threshold=5.0
        )

        self.get_logger().info(
            'Loop Closure Manager started.'
        )

    def handle_candidate(self, candidate):
        # Temporal verification can analyze every appearance hypothesis
        temporal_result = self.temporal_verifier.verify(candidate)

        # Graph verification only makes sense when RTAB-Map reports
        # an actual loop closure in this Info message
        if candidate.actual_loop_closure_id > 0:
            graph_result = self.graph_verifier.verify(candidate)

            graph_text = (
                '\n'
                '[Graph]\n'
                f'Score  : {graph_result.score:.3f}\n'
                f'Passed : {graph_result.passed}\n'
                f'Reason : {graph_result.reason}\n'
            )

        else:
            graph_text = (
                '\n'
                '[Graph]\n'
                'Skipped : No actual loop closure was reported '
                'for this update.\n'
            )

        self.get_logger().info(
            '\n'
            '======= Verification Results =======\n'
            f'Candidate : '
            f'{candidate.current_node_id} -> '
            f'{candidate.matched_node_id}\n'
            '\n'
            '[RTAB-Map]\n'
            f'Accepted hypothesis : '
            f'{candidate.accepted_hypothesis_id}\n'
            f'Actual loop closure : '
            f'{candidate.actual_loop_closure_id}\n'
            '\n'
            '[Temporal]\n'
            f'Score  : {temporal_result.score:.3f}\n'
            f'Passed : {temporal_result.passed}\n'
            f'Reason : {temporal_result.reason}\n'
            f'{graph_text}'
            '===================================='
        )


def main(args=None):
    rclpy.init(args=args)

    node = LoopClosureManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()