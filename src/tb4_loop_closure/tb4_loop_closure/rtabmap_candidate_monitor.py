from typing import Callable, Optional

from rtabmap_msgs.msg import Info

from tb4_loop_closure.loop_candidate import LoopCandidate


class RTABMapCandidateMonitor:

    def __init__(
        self,
        node,
        candidate_callback: Optional[Callable[[LoopCandidate], None]] = None
    ):
        self.node = node
        self.candidate_callback = candidate_callback

        self.subscription = self.node.create_subscription(
            Info,
            '/info',
            self.info_callback,
            10
        )

        self.node.get_logger().info(
            'RTAB-Map candidate monitor started.'
        )

    def info_callback(self, msg: Info):
        stats = self._stats_to_dict(msg)

        current_id = msg.ref_id

        hypothesis_id = int(
            stats.get(
                'Loop/Highest_hypothesis_id/',
                0.0
            )
        )

        hypothesis_value = float(
            stats.get(
                'Loop/Highest_hypothesis_value/',
                0.0
            )
        )

        accepted_id = int(
            stats.get(
                'Loop/Accepted_hypothesis_id/',
                0.0
            )
        )

        rejected = bool(
            stats.get(
                'Loop/RejectedHypothesis/',
                0.0
            )
        )

        # No loop candidate in this update
        if hypothesis_id <= 0:
            return

        candidate = LoopCandidate(
            current_node_id=current_id,
            matched_node_id=hypothesis_id,

            hypothesis_value=hypothesis_value,

            visual_matches=int(
                stats.get(
                    'Loop/Visual_matches/',
                    0.0
                )
            ),

            visual_inliers=int(
                stats.get(
                    'Loop/Visual_inliers/',
                    0.0
                )
            ),

            visual_inliers_ratio=float(
                stats.get(
                    'Loop/Visual_inliers_ratio/',
                    0.0
                )
            ),

            visual_inliers_distribution=float(
                stats.get(
                    'Loop/Visual_inliers_distribution/',
                    0.0
                )
            ),

            optimization_error=float(
                stats.get(
                    'Loop/Optimization_error/',
                    0.0
                )
            ),

            optimization_max_error=float(
                stats.get(
                    'Loop/Optimization_max_error/',
                    0.0
                )
            ),

            optimization_max_error_ratio=float(
                stats.get(
                    'Loop/Optimization_max_error_ratio/',
                    0.0
                )
            ),

            optimization_max_ang_error_ratio=float(
                stats.get(
                    'Loop/Optimization_max_ang_error_ratio/',
                    0.0
                )
            ),

            accepted_hypothesis_id=accepted_id,
            actual_loop_closure_id=msg.loop_closure_id,

            rejected=rejected,

            relative_transform=msg.loop_closure_transform
        )

        self.node.get_logger().info(
            '\n'
            '========== Loop Candidate ==========\n'
            f'Current node      : {candidate.current_node_id}\n'
            f'Matched node      : {candidate.matched_node_id}\n'
            f'Hypothesis score  : {candidate.hypothesis_value:.4f}\n'
            f'Visual matches    : {candidate.visual_matches}\n'
            f'Visual inliers    : {candidate.visual_inliers}\n'
            f'Inlier ratio      : {candidate.visual_inliers_ratio:.4f}\n'
            f'Inlier distribution: '
            f'{candidate.visual_inliers_distribution:.4f}\n'
            f'Optimization error: '
            f'{candidate.optimization_error:.4f}\n'
            f'Optimization max angular ratio: '
            f'{candidate.optimization_max_ang_error_ratio:.4f}\n'
            f'Highest hypothesis : {candidate.matched_node_id}\n'
            f'Accepted hypothesis: {candidate.accepted_hypothesis_id}\n'
            f'Actual loop closure: {candidate.actual_loop_closure_id}\n'
            f'Rejected          : {candidate.rejected}\n'
            '===================================='
        )

        if self.candidate_callback is not None:
            self.candidate_callback(candidate)

    @staticmethod
    def _stats_to_dict(msg: Info) -> dict:
        return {
            key: value
            for key, value in zip(
                msg.stats_keys,
                msg.stats_values
            )
        }