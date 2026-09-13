"""Known-noise filters for /rosout events."""

# 已知噪音：实车未使用 can1，socket_can1_receiver 的超时可忽略
_ROSOUT_NOISE_PATTERNS = (
    "can1 - CAN Receive Timeout",
    "can1 - CAN Receive",
)


def _is_rosout_noise(node: str, msg_text: str) -> bool:
    """过滤已知无意义的 /rosout 事件（如未接线的 can1 超时）。"""
    text = f"{node} {msg_text}".lower()
    if "can1" in text and "can receive timeout" in text:
        return True
    return any(p.lower() in text for p in _ROSOUT_NOISE_PATTERNS)

