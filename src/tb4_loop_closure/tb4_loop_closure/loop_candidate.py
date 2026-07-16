from dataclasses import dataclass
from typing import Optional


@dataclass
class LoopCandidate:
    current_node_id: int
    matched_node_id: int

    hypothesis_value: float = 0.0

    visual_matches: int = 0
    visual_inliers: int = 0
    visual_inliers_ratio: float = 0.0
    visual_inliers_distribution: float = 0.0

    optimization_error: float = 0.0
    optimization_max_error: float = 0.0
    optimization_max_error_ratio: float = 0.0
    optimization_max_ang_error_ratio: float = 0.0

    accepted_hypothesis_id: int = 0
    actual_loop_closure_id: int = 0

    rejected: bool = False

    relative_transform: Optional[object] = None