from tb4_loop_closure.verifiers.base_verifier import BaseVerifier
from tb4_loop_closure.verification_result import VerificationResult


class GraphConsistencyVerifier(BaseVerifier):

    def __init__(
        self,
        warning_error_threshold=20.0,
        reject_error_threshold=100.0,
        max_error_ratio_threshold=5.0,
        max_ang_error_ratio_threshold=5.0
    ):
        self.warning_error_threshold = warning_error_threshold
        self.reject_error_threshold = reject_error_threshold
        self.max_error_ratio_threshold = max_error_ratio_threshold
        self.max_ang_error_ratio_threshold = (
            max_ang_error_ratio_threshold
        )

    def verify(self, candidate):
        error = candidate.optimization_error
        error_ratio = candidate.optimization_max_error_ratio
        angular_ratio = (
            candidate.optimization_max_ang_error_ratio
        )

        # No optimization result available yet
        if (
            error == 0.0
            and error_ratio == 0.0
            and angular_ratio == 0.0
        ):
            return VerificationResult(
                verifier_name='graph',
                score=0.5,
                passed=True,
                reason='No non-zero graph consistency statistics available for this update.'
            )

        reasons = []

        if error >= self.reject_error_threshold:
            reasons.append(
                f'optimization error too large: {error:.3f}'
            )

        if error_ratio >= self.max_error_ratio_threshold:
            reasons.append(
                f'linear error ratio too large: '
                f'{error_ratio:.3f}'
            )

        if angular_ratio >= self.max_ang_error_ratio_threshold:
            reasons.append(
                f'angular error ratio too large: '
                f'{angular_ratio:.3f}'
            )

        if reasons:
            return VerificationResult(
                verifier_name='graph',
                score=0.0,
                passed=False,
                reason='; '.join(reasons)
            )

        # Convert optimization error into a rough confidence score
        if error <= self.warning_error_threshold:
            score = 1.0
        else:
            span = (
                self.reject_error_threshold
                - self.warning_error_threshold
            )

            score = 1.0 - (
                error - self.warning_error_threshold
            ) / span

            score = max(0.0, min(1.0, score))

        return VerificationResult(
            verifier_name='graph',
            score=score,
            passed=True,
            reason=(
                f'optimization_error={error:.3f}, '
                f'linear_ratio={error_ratio:.3f}, '
                f'angular_ratio={angular_ratio:.3f}'
            )
        )