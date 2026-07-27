import numpy as np

from .trajectory_aligner import TrajectoryAligner


class TrajectoryPriorConstraint:
    """
    Compute the Trajectory Prior Constraint (TPC) score.

    The score corresponds to the RMSE between:

        P

    and the aligned:

        P_star

    Lower score:
        trajectory deformation is small

    Higher score:
        trajectory deformation is large
    """

    def __init__(
        self,
        threshold: float,
    ):
        if threshold <= 0:
            raise ValueError(
                "threshold must be greater than 0"
            )

        self.threshold = threshold

    def compute_score(
        self,
        trajectory_before: np.ndarray,
        trajectory_after: np.ndarray,
    ) -> dict:
        """
        Compute TPC score.

        Args:
            trajectory_before:
                Nx3 translation trajectory P.

            trajectory_after:
                Nx3 optimized trajectory P_star.

        Returns:
            Dictionary containing:

                score
                passed
                aligned_trajectory
                rotation
                translation
        """

        (
            aligned_trajectory,
            rotation,
            translation,
        ) = TrajectoryAligner.align(
            reference=trajectory_before,
            target=trajectory_after,
        )

        trajectory_before = np.asarray(
            trajectory_before,
            dtype=np.float64,
        )

        # ---------------------------------------------
        # Equation (6)
        #
        # s =
        # sqrt(
        #     1/N *
        #     sum(
        #         || p_i - p_i_aligned ||^2
        #     )
        # )
        # ---------------------------------------------

        difference = (
            trajectory_before
            - aligned_trajectory
        )

        squared_distance = np.sum(
            difference ** 2,
            axis=1,
        )

        score = float(
            np.sqrt(
                np.mean(
                    squared_distance
                )
            )
        )

        passed = (
            score
            <= self.threshold
        )

        return {
            'score': score,
            'passed': passed,
            'aligned_trajectory': aligned_trajectory,
            'rotation': rotation,
            'translation': translation,
        }