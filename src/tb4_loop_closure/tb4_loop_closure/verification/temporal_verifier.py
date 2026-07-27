from collections import deque

from tb4_loop_closure.verifiers.base_verifier import BaseVerifier
from tb4_loop_closure.verification_result import VerificationResult


class TemporalVerifier(BaseVerifier):

    def __init__(
        self,
        window_size=5,
        min_consistent_ratio=0.6,
        max_matched_stagnation=2
    ):
        self.window_size = window_size
        self.min_consistent_ratio = min_consistent_ratio
        self.max_matched_stagnation = max_matched_stagnation

        self.history = deque(maxlen=window_size)

    def verify(self, candidate):
        self.history.append(candidate)

        if len(self.history) < 3:
            return VerificationResult(
                verifier_name='temporal',
                score=0.5,
                passed=True,
                reason='Not enough temporal history yet.'
            )

        matched_ids = [
            c.matched_node_id
            for c in self.history
        ]

        current_ids = [
            c.current_node_id
            for c in self.history
        ]

        # 1. 檢查 matched node 是否長時間卡在同一區域
        matched_span = max(matched_ids) - min(matched_ids)

        if matched_span <= self.max_matched_stagnation:
            return VerificationResult(
                verifier_name='temporal',
                score=0.1,
                passed=False,
                reason=(
                    f'Matched nodes are nearly stagnant: '
                    f'{matched_ids}'
                )
            )

        # 2. 檢查 current 與 matched sequence 是否大致同方向變化
        consistent_count = 0
        total_pairs = 0

        for i in range(1, len(self.history)):
            current_delta = current_ids[i] - current_ids[i - 1]
            matched_delta = matched_ids[i] - matched_ids[i - 1]

            if current_delta == 0:
                continue

            total_pairs += 1

            if current_delta * matched_delta > 0:
                consistent_count += 1

        if total_pairs == 0:
            consistency_ratio = 0.0
        else:
            consistency_ratio = consistent_count / total_pairs

        passed = consistency_ratio >= self.min_consistent_ratio

        return VerificationResult(
            verifier_name='temporal',
            score=consistency_ratio,
            passed=passed,
            reason=(
                f'current={current_ids}, '
                f'matched={matched_ids}, '
                f'consistency={consistency_ratio:.2f}'
            )
        )