from abc import ABC, abstractmethod

from tb4_loop_closure.loop_candidate import LoopCandidate
from tb4_loop_closure.verification_result import VerificationResult


class BaseVerifier(ABC):

    @abstractmethod
    def verify(self, candidate: LoopCandidate) -> VerificationResult:
        pass