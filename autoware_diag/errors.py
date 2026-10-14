"""Shared analysis errors and response-size helpers."""
from __future__ import annotations

from typing import Any

INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603
MAX_RESPONSE_CHARS = 100_000


class AnalysisError(Exception):
    def __init__(self, code: int, message: str, data: Any = None):
        self.code = code
        self.message = message
        self.data = data
        super().__init__(message)


def _truncate_text(text: str, max_chars: int = MAX_RESPONSE_CHARS, label: str = "response") -> str:
    if len(text) <= max_chars:
        return text
    cut = text[:max_chars]
    last_nl = cut.rfind("\n")
    if last_nl > max_chars // 2:
        cut = text[:last_nl]
    return (
        cut
        + f"\n\n--- [TRUNCATED: {label} exceeded {max_chars} chars "
        f"({len(text)} total); {len(text) - max_chars} chars omitted] ---\n"
    )
