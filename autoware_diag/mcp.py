"""MCP JSON-RPC over stdio protocol implementation."""
from __future__ import annotations

import json
import sys
import traceback
from typing import Any, Callable, Dict, List, Optional

from .log import log

PARSE_ERROR = -32700
INVALID_REQUEST = -32600
METHOD_NOT_FOUND = -32601
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603

# 最大响应输出长度（字符数），超过此长度截断
MAX_RESPONSE_CHARS = 100_000


def _truncate_text(text: str, max_chars: int = MAX_RESPONSE_CHARS, label: str = "response") -> str:
    """Truncate text at max_chars with a notice, keeping JSON valid-ish."""
    if len(text) <= max_chars:
        return text
    cut = text[:max_chars]
    # Try to cut at last newline before limit so partial lines don't break display
    last_nl = cut.rfind("\n")
    if last_nl > max_chars // 2:
        cut = text[:last_nl]
    return (
        cut
        + f'\n\n--- [TRUNCATED: {label} exceeded {max_chars} chars '
        f'({len(text)} total); {len(text) - max_chars} chars omitted] ---\n'
    )


class McpError(Exception):
    def __init__(self, code: int, message: str, data: Any = None):
        self.code = code
        self.message = message
        self.data = data
        super().__init__(message)


class McpServer:
    """Minimal MCP server implementing JSON-RPC 2.0 over stdio."""

    def __init__(
        self,
        name: str = "autoware-diag-mcp",
        version: str = "1.0.0",
        instructions: str = "",
    ):
        self.name = name
        self.version = version
        self.instructions = instructions
        self._handlers: Dict[str, Callable] = {}
        self._initialized = False
        self._register("initialize", self._handle_initialize)
        # MCP 客户端会定期 ping 保活，必须应答空 result
        self._register("ping", lambda params: {})

    def _register(self, method: str, handler: Callable):
        self._handlers[method] = handler

    def tool(self, name: str, description: str = "", input_schema: Optional[Dict] = None):
        """Decorator to register a tool handler."""
        def decorator(func):
            self._handlers[f"tools/call:{name}"] = func
            if not hasattr(self, "_tools"):
                self._tools: List[Dict] = []
            self._tools.append(
                {
                    "name": name,
                    "description": description,
                    "inputSchema": input_schema
                    or {"type": "object", "properties": {}},
                }
            )
            return func
        return decorator

    def _handle_initialize(self, params: Dict) -> Dict:
        self._initialized = True
        return {
            "protocolVersion": "2025-03-26",
            "capabilities": {
                "tools": {
                    "listChanged": False,
                },
            },
            "serverInfo": {
                "name": self.name,
                "version": self.version,
            },
            "instructions": self.instructions,
        }

    def _send_message(self, msg: Dict):
        line = json.dumps(msg, ensure_ascii=False, default=str)
        sys.stdout.write(line + "\n")
        sys.stdout.flush()

    def _handle_request(self, request: Dict):
        msg_id = request.get("id")
        method = request.get("method", "")
        params = request.get("params", {}) or {}

        try:
            if method == "tools/list":
                tools = getattr(self, "_tools", [])
                result = {"tools": tools}
                self._send_message(
                    {
                        "jsonrpc": "2.0",
                        "id": msg_id,
                        "result": result,
                    }
                )

            elif method == "tools/call":
                tool_name = params.get("name", "")
                arguments = params.get("arguments", {})
                handler_key = f"tools/call:{tool_name}"

                if handler_key not in self._handlers:
                    self._send_message(
                        {
                            "jsonrpc": "2.0",
                            "id": msg_id,
                            "error": {
                                "code": METHOD_NOT_FOUND,
                                "message": f"Unknown tool: {tool_name}",
                            },
                        }
                    )
                    return

                try:
                    result = self._handlers[handler_key](arguments)
                    if isinstance(result, dict) and "content" in result:
                        # 对已存在的 content 做大小截断
                        content = result.get("content", [])
                        truncated_content = []
                        for item in content:
                            if isinstance(item, dict) and item.get("type") == "text":
                                item["text"] = _truncate_text(
                                    item["text"], label=f"tool {tool_name}"
                                )
                            truncated_content.append(item)
                        result["content"] = truncated_content
                        self._send_message(
                            {
                                "jsonrpc": "2.0",
                                "id": msg_id,
                                "result": result,
                            }
                        )
                    else:
                        text = json.dumps(result, ensure_ascii=False, default=str)
                        text = _truncate_text(text, label=f"tool {tool_name}")
                        self._send_message(
                            {
                                "jsonrpc": "2.0",
                                "id": msg_id,
                                "result": {
                                    "content": [
                                        {
                                            "type": "text",
                                            "text": text,
                                        }
                                    ],
                                },
                            }
                        )
                except McpError as e:
                    self._send_message(
                        {
                            "jsonrpc": "2.0",
                            "id": msg_id,
                            "result": {
                                "content": [{"type": "text", "text": e.message}],
                                "isError": True,
                            },
                        }
                    )
                except Exception as e:
                    self._send_message(
                        {
                            "jsonrpc": "2.0",
                            "id": msg_id,
                            "result": {
                                "content": [
                                    {
                                        "type": "text",
                                        "text": f"Error: {str(e)}\n{traceback.format_exc()}",
                                    }
                                ],
                                "isError": True,
                            },
                        }
                    )

            elif method in self._handlers:
                result = self._handlers[method](params)
                self._send_message(
                    {
                        "jsonrpc": "2.0",
                        "id": msg_id,
                        "result": result,
                    }
                )

            else:
                self._send_message(
                    {
                        "jsonrpc": "2.0",
                        "id": msg_id,
                        "error": {
                            "code": METHOD_NOT_FOUND,
                            "message": f"Method not found: {method}",
                        },
                    }
                )

        except McpError as e:
            self._send_message(
                {
                    "jsonrpc": "2.0",
                    "id": msg_id,
                    "error": {"code": e.code, "message": e.message, "data": e.data},
                }
            )
        except Exception as e:
            self._send_message(
                {
                    "jsonrpc": "2.0",
                    "id": msg_id,
                    "error": {
                        "code": INTERNAL_ERROR,
                        "message": str(e),
                        "data": {"traceback": traceback.format_exc()},
                    },
                }
            )

    def _handle_notification(self, notification: Dict):
        method = notification.get("method", "")
        if method == "notifications/initialized":
            self._initialized = True
        elif method in ("notifications/cancelled", "$/cancellation"):
            # MCP 规范名为 notifications/cancelled；$/cancellation 是 LSP 写法，容忍并存
            pass

    def _process_message(self, raw: str):
        if not raw.strip():
            return
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            self._send_message(
                {
                    "jsonrpc": "2.0",
                    "id": None,
                    "error": {"code": PARSE_ERROR, "message": "Parse error"},
                }
            )
            return
        if isinstance(msg, list):
            for item in msg:
                self._process_single(item)
            return
        self._process_single(msg)

    def _process_single(self, msg: Dict):
        if "id" in msg and msg["id"] is not None:
            self._handle_request(msg)
        else:
            self._handle_notification(msg)

    def run(self):
        sys.stderr.write(f"Autoware Diag MCP server starting (protocol 2025-03-26)...\n")
        sys.stderr.flush()
        # stdio 传输强制 UTF-8，避免跟随 locale 编码
        for stream in (sys.stdin, sys.stdout):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass
        # MCP stdio 是 NDJSON：每条消息一行完整的 JSON-RPC，不允许内嵌换行。
        # 直接逐行解析 —— 不要按括号配平累积（字符串里含不配平括号会吞掉合法请求）。
        for line in sys.stdin:
            try:
                raw = line.strip()
                if not raw:
                    # Skip empty lines (keepalive / whitespace)
                    continue
                if len(raw) > 64 * 1024 * 1024:  # 64MB safety limit
                    self._send_message(
                        {
                            "jsonrpc": "2.0",
                            "id": None,
                            "error": {
                                "code": INVALID_REQUEST,
                                "message": "Message too large (>64MB)",
                            },
                        }
                    )
                    continue
                self._process_message(raw)
            except SystemExit:
                break
            except Exception:
                sys.stderr.write(
                    f"Error processing message: {traceback.format_exc()}\n"
                )
                sys.stderr.flush()

