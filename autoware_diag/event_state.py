"""Shared streaming state transitions for legacy analyzers and quick triage.

Sinks implement append/add; they may retain bounded examples without discarding counts.
"""
from .core import _time_str
from .parsers import (
    _val,
    _AEB_REQ_LABELS, _AEB_SENSITIVITY_LABELS, _parse_aeb_shadow,
)


def update_shadow(data, log_time_ns, prev, aeb_events, aeb_timeline):
    p = _parse_aeb_shadow(data)
    aeb_req = bool(p["aeb_req_filter"]) if p["aeb_req_filter"] is not None else None
    fcw_req = bool(p["fcw_req_filter"]) if p["fcw_req_filter"] is not None else None
    aeb_raw = bool(p["aeb_req_no_filter"]) if p["aeb_req_no_filter"] is not None else None
    fcw_raw = bool(p["fcw_req_no_filter"]) if p["fcw_req_no_filter"] is not None else None
    aeb_status = p["aeb_status"]
    fcw_status = p["fcw_status"]

    # AEB 触发（滤波后请求）
    if aeb_req and prev["aeb_req"] is not True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "AEB_ACTIVATED",
            "severity": "error",
            "detail": (
                "AEB request activated (filtered) "
                f"[no_filter={int(bool(aeb_raw))}, status={p['aeb_status_label']}]"
                + (" (already active at recording start)" if prev["aeb_req"] is None else "")
            ),
        })
    elif aeb_req is False and prev["aeb_req"] is True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "AEB_CLEARED",
            "severity": "info",
            "detail": "AEB request cleared",
        })

    # FCW 触发（滤波后请求）
    if fcw_req and prev["fcw_req"] is not True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "FCW_ACTIVATED",
            "severity": "warning",
            "detail": (
                "FCW request activated (filtered) "
                f"[no_filter={int(bool(fcw_raw))}, status={p['fcw_status_label']}]"
                + (" (already active at recording start)" if prev["fcw_req"] is None else "")
            ),
        })
    elif fcw_req is False and prev["fcw_req"] is True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "FCW_CLEARED",
            "severity": "info",
            "detail": "FCW request cleared",
        })

    # 原始请求被滤波拦截 — 诊断「AEB 该触发未触发」的关键线索。
    # 结合官方语义区分两种根因：
    #   功能 OFF(0) → 请求未生效属设计预期（功能未开启）；
    #   功能 ON(1) 却被拦 → 疑似滤波逻辑问题，需深查。
    if aeb_raw and not aeb_req and prev["aeb_raw"] is not True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "AEB_REQUEST_FILTERED",
            "severity": "warning",
            "detail": (
                "AEB raw request (no_filter=1) was filtered out (filter=0) — AEB did not engage"
                + (
                    " [aeb_status=OFF: feature disabled, request not executed by design]"
                    if aeb_status == 0
                    else (
                        " [aeb_status=ON: feature enabled but request filtered — check filter logic]"
                        if aeb_status == 1
                        else ""
                    )
                )
                + (" (already the case at recording start)" if prev["aeb_raw"] is None else "")
            ),
        })
    if fcw_raw and not fcw_req and prev["fcw_raw"] is not True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "FCW_REQUEST_FILTERED",
            "severity": "warning",
            "detail": (
                "FCW raw request (no_filter=1) was filtered out (filter=0) — FCW did not engage"
                + (
                    " [fcw_status=OFF: feature disabled, request not executed by design]"
                    if fcw_status == 0
                    else (
                        " [fcw_status=ON: feature enabled but request filtered — check filter logic]"
                        if fcw_status == 1
                        else ""
                    )
                )
                + (" (already the case at recording start)" if prev["fcw_raw"] is None else "")
            ),
        })

    # status 故障（0=OFF 1=ON 2=FAULT）
    if aeb_status == 2 and prev["aeb_fault"] is not True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "AEB_STATUS_FAULT",
            "severity": "error",
            "detail": "AEB status=FAULT"
            + (" (already fault at recording start)" if prev["aeb_fault"] is None else ""),
        })
    elif aeb_status is not None and aeb_status != 2 and prev["aeb_fault"] is True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "AEB_STATUS_RECOVERED",
            "severity": "info",
            "detail": "AEB status recovered from FAULT",
        })
    if fcw_status == 2 and prev["fcw_fault"] is not True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "FCW_STATUS_FAULT",
            "severity": "error",
            "detail": "FCW status=FAULT"
            + (" (already fault at recording start)" if prev["fcw_fault"] is None else ""),
        })
    elif fcw_status is not None and fcw_status != 2 and prev["fcw_fault"] is True:
        aeb_events.append({
            "time": _time_str(log_time_ns),
            "event": "FCW_STATUS_RECOVERED",
            "severity": "info",
            "detail": "FCW status recovered from FAULT",
        })

    prev.update({
        "aeb_req": aeb_req, "fcw_req": fcw_req,
        "aeb_raw": aeb_raw, "fcw_raw": fcw_raw,
        "aeb_fault": aeb_status == 2, "fcw_fault": fcw_status == 2,
    })

    if aeb_timeline is not None:
        aeb_timeline.add({
            "time": _time_str(log_time_ns),
            "aeb_status": p["aeb_status_label"],
            "fcw_status": p["fcw_status_label"],
            "aeb_req_filter": _AEB_REQ_LABELS.get(
                p["aeb_req_filter"], p["aeb_req_filter"]
            ),
            "fcw_req_filter": _AEB_REQ_LABELS.get(
                p["fcw_req_filter"], p["fcw_req_filter"]
            ),
            "aeb_req_no_filter": _AEB_REQ_LABELS.get(
                p["aeb_req_no_filter"], p["aeb_req_no_filter"]
            ),
            "fcw_req_no_filter": _AEB_REQ_LABELS.get(
                p["fcw_req_no_filter"], p["fcw_req_no_filter"]
            ),
            "aeb_sensitivity_auto": _AEB_SENSITIVITY_LABELS.get(
                p["aeb_sensitivity_auto"], p["aeb_sensitivity_auto"]
            ),
            "aeb_sensitivity_local": _AEB_SENSITIVITY_LABELS.get(
                p["aeb_sensitivity_local"], p["aeb_sensitivity_local"]
            ),
            "fcw_sensitivity_auto": _AEB_SENSITIVITY_LABELS.get(
                p["fcw_sensitivity_auto"], p["fcw_sensitivity_auto"]
            ),
            "fcw_sensitivity_local": _AEB_SENSITIVITY_LABELS.get(
                p["fcw_sensitivity_local"], p["fcw_sensitivity_local"]
            ),
        })


def update_legacy(data, log_time_ns, prev, aeb_events, aeb_timeline):
    prev_aeb_active = prev.get("aeb")
    prev_fcw_active = prev.get("fcw")
    d = data
    aeb_active = bool(
        _val(d, "acu_aeb_active_request")
        or _val(d, "acu_aeb_brake_en")
    )
    aeb_brake_en = _val(d, "acu_aeb_brake_en")
    if aeb_brake_en is None and not hasattr(d, "acu_aeb_brake_en"):
        aeb_brake_en = False
    fcw_active = bool(_val(d, "acu_fcw_active_request"))
    brake_pedal = d.get("acu_aeb_brake_pedal_target", 0) if isinstance(d, dict) else getattr(d, "acu_aeb_brake_pedal_target", 0)

    # Detect transitions（None 表示第一条消息：若已激活则标注录制开始即激活）
    if aeb_active and prev_aeb_active is not True:
        aeb_events.append(
            {
                "time": _time_str(log_time_ns),
                "event": "AEB_ACTIVATED",
                "brake_pedal_target": brake_pedal,
                "severity": "error",
                "detail": "AEB activated"
                + (" (already active at recording start)" if prev_aeb_active is None else ""),
            }
        )
    if fcw_active and prev_fcw_active is not True:
        aeb_events.append(
            {
                "time": _time_str(log_time_ns),
                "event": "FCW_ACTIVATED",
                "severity": "warning",
                "detail": "FCW activated"
                + (" (already active at recording start)" if prev_fcw_active is None else ""),
            }
        )

    prev_aeb_active = aeb_active
    prev_fcw_active = fcw_active

    if aeb_timeline is not None:
        aeb_timeline.add(
            {
                "time": _time_str(log_time_ns),
                "aeb_active": aeb_active,
                "aeb_brake_enabled": aeb_brake_en,
                "fcw_active": fcw_active,
                "brake_pedal_target": brake_pedal,
                "work_status": _val(d, "acu_aeb_work_sta"),
            }
        )
    prev.update(aeb=prev_aeb_active, fcw=prev_fcw_active)


def update_mrm(sig_type, ts_ns, val, prev, events):
    prev_engage = prev.get("engage")
    prev_emergency = prev.get("emergency_flag")
    prev_emg_cmd = prev.get("emergency_cmd")
    if val is None:
        return
    if sig_type == "engage":
        if prev_engage is not None and prev_engage != val:
            if val is False:
                events.append(
                    {
                        "time": _time_str(ts_ns),
                        "event": "DISENGAGE",
                        "detail": "Autoware disengaged (MRM triggered)",
                        "severity": "error",
                    }
                )
            else:
                events.append(
                    {
                        "time": _time_str(ts_ns),
                        "event": "ENGAGE",
                        "detail": "Autoware engaged",
                        "severity": "info",
                    }
                )
        prev_engage = val
    elif sig_type == "emergency_flag":
        if val is True and prev_emergency is not True:
            events.append(
                {
                    "time": _time_str(ts_ns),
                    "event": "EMERGENCY_ON",
                    "detail": "Emergency flag activated"
                    + (" (already active at recording start)" if prev_emergency is None else ""),
                    "severity": "error",
                }
            )
        elif val is False and prev_emergency is True:
            events.append(
                {
                    "time": _time_str(ts_ns),
                    "event": "EMERGENCY_OFF",
                    "detail": "Emergency flag deactivated",
                    "severity": "info",
                }
            )
        prev_emergency = val
    elif sig_type == "emergency_cmd":
        if val is True and prev_emg_cmd is not True:
            events.append(
                {
                    "time": _time_str(ts_ns),
                    "event": "EMERGENCY_CMD_ON",
                    "detail": "Emergency control command activated (vehicle_cmd.emergency=True)"
                    + (" (already active at recording start)" if prev_emg_cmd is None else ""),
                    "severity": "error",
                }
            )
        elif val is False and prev_emg_cmd is True:
            events.append(
                {
                    "time": _time_str(ts_ns),
                    "event": "EMERGENCY_CMD_OFF",
                    "detail": "Emergency control command cleared",
                    "severity": "info",
                }
            )
        prev_emg_cmd = val
    prev.update(engage=prev_engage, emergency_flag=prev_emergency, emergency_cmd=prev_emg_cmd)
