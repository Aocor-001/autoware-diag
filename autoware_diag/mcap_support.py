"""Optional mcap / mcap-ros2 imports and CDR decoder factory."""
from __future__ import annotations

from typing import Callable, Dict, Optional

HAVE_MCAP = False
try:
    from mcap.reader import make_reader
    from mcap.decoder import DecoderFactory as McapDecoderFactory
    from mcap.records import Schema as McapSchema

    HAVE_MCAP = True
except ImportError:
    make_reader = None  # type: ignore
    McapDecoderFactory = object  # type: ignore
    McapSchema = object  # type: ignore

HAVE_ROS2 = False
try:
    from mcap_ros2.decoder import generate_dynamic

    HAVE_ROS2 = True
except ImportError:
    generate_dynamic = None  # type: ignore


class Ros2DecoderFactory(McapDecoderFactory):
    """Decode ROS2 CDR-encoded MCAP messages using mcap-ros2-support."""

    def __init__(self):
        self._decoders: Dict[str, Dict[str, Callable]] = {}

    def decoder_for(self, message_encoding: str, schema: Optional[McapSchema]):
        if message_encoding != "cdr":
            return None
        if not schema or not schema.data:
            return None
        schema_name = schema.name
        schema_text = schema.data.decode("utf-8")

        if schema_name not in self._decoders:
            try:
                decoders = generate_dynamic(schema_name, schema_text)
                self._decoders[schema_name] = decoders
            except Exception:
                return None

        decoders = self._decoders.get(schema_name, {})
        return decoders.get(schema_name)

