import numpy as np


class TrajectoryAligner:
    """
    Align two trajectories using rigid SE(3) transformation.

    We find R and t such that:

        P ~= R @ P_star + t

    where:
        P      : reference trajectory before PGO
        P_star : trajectory after hypothetical PGO
    """

    @staticmethod
    def align(
        reference: np.ndarray,
        target: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Align target trajectory to reference trajectory.

        Args:
            reference:
                Nx3 trajectory before optimization.

            target:
                Nx3 trajectory after hypothetical optimization.

        Returns:
            aligned_target:
                Nx3 aligned target trajectory.

            rotation:
                3x3 rotation matrix R.

            translation:
                3D translation vector t.
        """

        reference = np.asarray(reference, dtype=np.float64)
        target = np.asarray(target, dtype=np.float64)

        if reference.ndim != 2 or reference.shape[1] != 3:
            raise ValueError(
                "reference must have shape (N, 3)"
            )

        if target.ndim != 2 or target.shape[1] != 3:
            raise ValueError(
                "target must have shape (N, 3)"
            )

        if reference.shape != target.shape:
            raise ValueError(
                "reference and target must have the same shape"
            )

        if len(reference) < 3:
            raise ValueError(
                "At least 3 trajectory points are required"
            )

        # -------------------------------------------------
        # Step 1: Compute trajectory centroids
        # -------------------------------------------------

        reference_mean = np.mean(
            reference,
            axis=0
        )

        target_mean = np.mean(
            target,
            axis=0
        )

        # -------------------------------------------------
        # Step 2: Remove translation
        # -------------------------------------------------

        reference_centered = (
            reference
            - reference_mean
        )

        target_centered = (
            target
            - target_mean
        )

        # -------------------------------------------------
        # Step 3: Compute covariance matrix
        # -------------------------------------------------

        covariance = (
            target_centered.T
            @ reference_centered
        )

        # -------------------------------------------------
        # Step 4: SVD
        # -------------------------------------------------

        U, _, Vt = np.linalg.svd(
            covariance
        )

        rotation = (
            Vt.T
            @ U.T
        )

        # -------------------------------------------------
        # Step 5:
        # Prevent reflection
        #
        # det(R) must be +1 for a valid rotation.
        # -------------------------------------------------

        if np.linalg.det(rotation) < 0:
            Vt[-1, :] *= -1

            rotation = (
                Vt.T
                @ U.T
            )

        # -------------------------------------------------
        # Step 6: Compute translation
        #
        # reference ~= R * target + t
        # -------------------------------------------------

        translation = (
            reference_mean
            - rotation @ target_mean
        )

        # -------------------------------------------------
        # Step 7: Apply transformation
        # -------------------------------------------------

        aligned_target = (
            target
            @ rotation.T
            + translation
        )

        return (
            aligned_target,
            rotation,
            translation,
        )