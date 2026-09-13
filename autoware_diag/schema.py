"""Schema / custom message catalog tools."""
from __future__ import annotations

import json
from typing import Any, Dict, List

from .core import _dictify, _open_mcap, _require_file_path, _truncate_text
from .mcap_support import HAVE_ROS2
from .mcp import INTERNAL_ERROR, McpError
from .parsers import CUSTOM_MESSAGE_CATALOG

def describe_message_schema(args: Dict) -> Dict:
    file_path = _require_file_path(args)
    topic = args.get("topic")
    schema_name = args.get("schema_name")
    include_text = args.get("include_schema_text")
    if include_text is None:
        include_text = True
    if not topic and not schema_name:
        raise McpError(
            INVALID_PARAMS, "Provide topic or schema_name"
        )

    reader, fp = _open_mcap(file_path)
    try:
        summary = reader.get_summary()
    finally:
        fp.close()
    if not summary:
        raise McpError(INTERNAL_ERROR, "No summary section in MCAP")

    # Resolve schema
    matched_schemas = []
    topic_info = None
    if topic:
        for ch in summary.channels.values():
            if ch.topic == topic:
                sch = summary.schemas.get(ch.schema_id)
                topic_info = {
                    "topic": ch.topic,
                    "encoding": ch.message_encoding,
                    "schema_id": ch.schema_id,
                }
                if sch:
                    matched_schemas.append(sch)
                break
        if topic_info is None:
            raise McpError(INVALID_PARAMS, f"Topic not found in MCAP: {topic}")
    else:
        for sch in summary.schemas.values():
            if sch.name == schema_name or sch.name.endswith("/" + schema_name):
                matched_schemas.append(sch)
        if not matched_schemas:
            # prefix match
            for sch in summary.schemas.values():
                if schema_name and schema_name in sch.name:
                    matched_schemas.append(sch)

    results = []
    for sch in matched_schemas[:5]:
        text = sch.data.decode("utf-8", "replace") if sch.data else ""
        top = text.split(
            "================================================================================"
        )[0].strip() if text else ""
        # 话题列表
        topics_using = [
            ch.topic
            for ch in summary.channels.values()
            if ch.schema_id == sch.id
        ]
        entry = {
            "schema_name": sch.name,
            "encoding": sch.encoding,
            "topics_using_schema": topics_using,
            "custom_semantics": CUSTOM_MESSAGE_CATALOG.get(sch.name),
        }
        if include_text:
            # 顶层定义优先；全文过长时截断嵌套部分
            entry["schema_top_level"] = top[:4000]
            if len(text) > 4000:
                entry["schema_full_length"] = len(text)
                entry["schema_full_truncated"] = text[:8000]
        results.append(entry)

    # Also list custom catalog hits when schema unknown
    catalog_notes = None
    if not results and schema_name:
        hits = {
            k: v
            for k, v in CUSTOM_MESSAGE_CATALOG.items()
            if schema_name.lower() in k.lower()
        }
        if hits:
            catalog_notes = hits

    return {
        "content": [
            {
                "type": "text",
                "text": json.dumps(
                    {
                        "file_path": file_path,
                        "topic": topic,
                        "schema_name_query": schema_name,
                        "topic_info": topic_info,
                        "schemas": results,
                        "catalog_notes": catalog_notes,
                        "custom_catalog_index": sorted(
                            CUSTOM_MESSAGE_CATALOG.keys()
                        ),
                        "note": (
                            "Pix/pix_robobus/aeb_interfaces/point_cloud_interfaces "
                            "为私有或改造消息，字段单位见 custom_semantics"
                        ),
                    },
                    ensure_ascii=False,
                    indent=2,
                    default=str,
                ),
            }
        ],
    }


def list_custom_messages(args: Dict) -> Dict:
    file_path = _require_file_path(args)
    reader, fp = _open_mcap(file_path)
    try:
        summary = reader.get_summary()
    finally:
        fp.close()
    if not summary:
        raise McpError(INTERNAL_ERROR, "No summary section in MCAP")

    def _is_custom(name: str) -> bool:
        n = (name or "").lower()
        return any(
            x in n
            for x in (
                "pix_",
                "pix/",
                "aeb_interfaces",
                "point_cloud_interfaces",
                "failcase",
            )
        )

    found = []
    ch_counts = {}
    try:
        ch_counts = (
            summary.statistics.channel_message_counts
            if summary.statistics and summary.statistics.channel_message_counts
            else {}
        )
    except Exception:
        pass

    for cid, ch in summary.channels.items():
        sch = summary.schemas.get(ch.schema_id)
        sname = sch.name if sch else ""
        if not (_is_custom(sname) or _is_custom(ch.topic)):
            continue
        found.append(
            {
                "topic": ch.topic,
                "schema": sname,
                "message_count": ch_counts.get(cid, 0),
                "encoding": ch.message_encoding,
                "semantics": CUSTOM_MESSAGE_CATALOG.get(sname),
            }
        )
    found.sort(key=lambda x: -x.get("message_count", 0))

    # catalog entries present in file even if topic filter missed
    present_schemas = {sch.name for sch in summary.schemas.values()}
    catalog_in_file = {
        k: v for k, v in CUSTOM_MESSAGE_CATALOG.items() if k in present_schemas
    }

    return {
        "content": [
            {
                "type": "text",
                "text": json.dumps(
                    {
                        "file_path": file_path,
                        "custom_topics": found,
                        "custom_topic_count": len(found),
                        "catalog_in_file": catalog_in_file,
                        "upstream_diff_summary": [
                            "pix_robobus_driver_msgs/* — Pix CAN 私有接口，转向角单位 deg",
                            "aeb_interfaces/AebInfoShadow — AEB/FCW 影子状态（原版无）",
                            "aeb_interfaces/AebObstacleDebugData — AEB 障碍物调试",
                            "point_cloud_interfaces/CompressedPointCloud2 — 压缩点云，非 PointCloud2",
                            "autoware_control_msgs/Control — 字段在 lateral/longitudinal 嵌套内",
                        ],
                    },
                    ensure_ascii=False,
                    indent=2,
                    default=str,
                ),
            }
        ],
    }

