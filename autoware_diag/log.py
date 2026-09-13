"""Shared logger."""
import logging
import sys

logging.basicConfig(
    level=logging.WARNING,
    format="[autoware-diag] %(levelname)s %(message)s",
    stream=sys.stderr,
)
log = logging.getLogger("autoware-diag")
