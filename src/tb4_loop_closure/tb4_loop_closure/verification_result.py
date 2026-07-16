from dataclasses import dataclass


@dataclass
class VerificationResult:
    verifier_name: str
    score: float
    passed: bool
    reason: str = ''