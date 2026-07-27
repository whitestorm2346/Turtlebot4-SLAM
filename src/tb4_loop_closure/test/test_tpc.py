import numpy as np

from tb4_loop_closure.verification.trajectory_prior_constraint import (
    TrajectoryPriorConstraint,
)


def test_identical_trajectory_score_is_zero():
    trajectory = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
    ])

    verifier = TrajectoryPriorConstraint(
        threshold=0.5
    )

    result = verifier.compute_score(
        trajectory,
        trajectory,
    )

    assert result['score'] < 1e-9
    assert result['passed'] is True


def test_translation_is_removed_by_alignment():
    trajectory_before = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
        [3.0, 1.0, 0.0],
    ])

    trajectory_after = trajectory_before + np.array([
        10.0,
        -5.0,
        2.0,
    ])

    verifier = TrajectoryPriorConstraint(
        threshold=0.5
    )

    result = verifier.compute_score(
        trajectory_before,
        trajectory_after,
    )

    assert result['score'] < 1e-9
    assert result['passed'] is True


def test_rotation_is_removed_by_alignment():
    trajectory_before = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
        [3.0, 1.0, 0.0],
    ])

    rotation_90_deg = np.array([
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0],
    ])

    trajectory_after = (
        trajectory_before
        @ rotation_90_deg.T
    )

    verifier = TrajectoryPriorConstraint(
        threshold=0.5
    )

    result = verifier.compute_score(
        trajectory_before,
        trajectory_after,
    )

    assert result['score'] < 1e-9
    assert result['passed'] is True


def test_deformed_trajectory_has_large_score():
    trajectory_before = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
    ])

    trajectory_after = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 2.0, 0.0],
        [3.0, -2.0, 0.0],
        [4.0, 3.0, 0.0],
    ])

    verifier = TrajectoryPriorConstraint(
        threshold=0.5
    )

    result = verifier.compute_score(
        trajectory_before,
        trajectory_after,
    )

    assert result['score'] > 0.5
    assert result['passed'] is False