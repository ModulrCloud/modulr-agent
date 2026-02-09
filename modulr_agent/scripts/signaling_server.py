"""
Mock Signaling Server for Local Testing (Python)

Mirrors the behavior of testing_new_signaling/robot-teleop-webapp/scripts/mock-signaling-server.ts

Usage:
    python signaling_server.py [port]

Default port: 8765
"""

import asyncio, websockets, json, ssl, uuid, sys, os
from datetime import datetime, timezone
from urllib.parse import urlparse, parse_qs


PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8765
LEGACY = os.environ.get("MOCK_LEGACY", "true").lower() != "false"
SUPPORTED_VERSIONS = ["0.0", "0.1"]


connections = {}        # connectionId -> {userId, kind, groups}
connection_protocol = {}  # connectionId -> 'legacy' | 'modulr-v0'
robot_presence = {}     # robotId -> {connectionId, ownerUserId, status}
ws_map = {}             # connectionId -> ws
ws_to_conn_id = {}      # ws -> connectionId (reverse lookup)

local = True

if local:
    ssl_context = None
else:
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain("certs/dev-server.crt", "certs/dev-server.key")

def make_envelope(msg_type, payload):
    return {
        "type": msg_type,
        "version": "0.0",
        "id": str(uuid.uuid4()),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "payload": payload,
    }


def normalize_new_protocol(msg):
    """Convert v0.0 envelope into a normalized internal dict, or None if unknown."""
    msg_type = msg.get("type", "")
    if "." not in msg_type:
        return None

    p = msg.get("payload") or {}
    t = msg_type.lower()

    if t == "signalling.register":
        agent_id = p.get("agentId", "").strip() or None
        return {"type": "register", "robotId": agent_id, "payload": p}

    if t == "signalling.offer":
        robot_id = next((x.strip() for x in [p.get("robotId"), p.get("agentId"), msg.get("robotId")] if isinstance(x, str) and x.strip()), None)
        conn_id = p.get("connectionId", "").strip() or None
        return {"type": "offer", "robotId": robot_id, "target": "robot", "clientConnectionId": conn_id,
                "payload": {"sdp": p.get("sdp"), "sdpType": p.get("sdpType", "offer")}}

    if t == "signalling.answer":
        robot_id = next((x.strip() for x in [p.get("robotId"), p.get("agentId")] if isinstance(x, str) and x.strip()), None)
        conn_id = p.get("connectionId", "").strip() or None
        return {"type": "answer", "robotId": robot_id, "target": "client", "clientConnectionId": conn_id,
                "payload": {"sdp": p.get("sdp"), "sdpType": p.get("sdpType", "answer")}}

    if t == "signalling.ice_candidate":
        robot_id = next((x.strip() for x in [p.get("robotId"), p.get("agentId")] if isinstance(x, str) and x.strip()), None)
        conn_id = p.get("connectionId", "").strip() or None
        return {"type": "ice-candidate", "robotId": robot_id, "clientConnectionId": conn_id,
                "payload": {"candidate": p.get("candidate"), "sdpMid": p.get("sdpMid"), "sdpMLineIndex": p.get("sdpMLineIndex")}}

    if t in ("agent.ping", "agent.pong"):
        return {"type": t}

    if t == "signalling.capabilities":
        return {"type": "signalling.capabilities", "payload": p}

    if t == "signalling.connected":
        conn_id = p.get("connectionId", "").strip() or None
        return {"type": "signalling.connected", "target": "client", "clientConnectionId": conn_id,
                "payload": p}

    if t == "signalling.disconnected":
        conn_id = p.get("connectionId", "").strip() or None
        return {"type": "signalling.disconnected", "target": "client", "clientConnectionId": conn_id,
                "payload": p}

    if t == "signalling.error":
        conn_id = p.get("connectionId", "").strip() or None
        return {"type": "signalling.error", "clientConnectionId": conn_id,
                "payload": p}

    return None


def format_outbound(msg, dest_protocol):
    """Wrap a normalized message into the destination's protocol format."""
    if dest_protocol != "modulr-v0":
        return msg

    envelope = {
        "type": msg.get("type"),
        "version": "0.0",
        "id": f"mock-{uuid.uuid4()}",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "payload": {},
    }
    payload = msg.get("payload") or {}
    t = msg.get("type")

    if t == "offer":
        envelope["type"] = "signalling.offer"
        envelope["payload"] = {
            "sdp": payload.get("sdp"),
            "sdpType": "offer",
            "connectionId": msg.get("from"),
        }
    elif t == "answer":
        envelope["type"] = "signalling.answer"
        envelope["payload"] = {
            "sdp": payload.get("sdp"),
            "sdpType": "answer",
            "connectionId": msg.get("to"),
        }
    elif t in ("candidate", "ice-candidate"):
        envelope["type"] = "signalling.ice_candidate"
        envelope["payload"] = {
            "candidate": payload.get("candidate") or msg.get("candidate"),
            "sdpMid": payload.get("sdpMid"),
            "sdpMLineIndex": payload.get("sdpMLineIndex", 0),
            "connectionId": msg.get("clientConnectionId") or msg.get("to") or msg.get("from"),
        }
    elif t in ("signalling.connected", "signalling.disconnected", "signalling.error"):
        envelope["type"] = t
        envelope["payload"] = payload

    return envelope


def is_ws_open(ws):
    """Check if a websocket connection is open (compatible with websockets v16+)."""
    try:
        from websockets.protocol import State
        return ws.state == State.OPEN
    except (ImportError, AttributeError):
        return getattr(ws, 'open', False)


def send_to(conn_id, data):
    """Queue a send to a connection by id. Returns a coroutine."""
    ws = ws_map.get(conn_id)
    if ws and is_ws_open(ws):
        return ws.send(json.dumps(data))
    return asyncio.sleep(0)  # no-op

def handle_register(connection_id, msg):
    robot_id = msg.get("robotId")
    if not robot_id:
        print(f"[{connection_id}] Register failed: robotId required")
        return

    conn = connections.get(connection_id)
    if conn:
        conn["kind"] = "robot"

    robot_presence[robot_id] = {
        "connectionId": connection_id,
        "ownerUserId": conn["userId"] if conn else connection_id,
        "status": "online",
    }
    print(f"[{connection_id}] Robot {robot_id} registered")


async def handle_signal(connection_id, msg):
    msg_type = msg.get("type")
    if not msg_type:
        print(f"[{connection_id}] Signal failed: type required")
        return

    # Is the sender a robot?
    is_from_robot = any(p["connectionId"] == connection_id for p in robot_presence.values())
    inferred_robot_id = None
    if is_from_robot:
        for rid, p in robot_presence.items():
            if p["connectionId"] == connection_id:
                inferred_robot_id = rid
                break

    robot_id = msg.get("robotId") or inferred_robot_id

    # If robotId is still unknown, check if clientConnectionId is a known robot
    if not robot_id and msg.get("clientConnectionId"):
        candidate = msg["clientConnectionId"]
        if candidate in robot_presence:
            robot_id = candidate

    # Determine target direction
    target = msg.get("target")
    if not target:
        if is_from_robot and msg.get("clientConnectionId"):
            target = "client"
        else:
            target = "robot"
    target = target.lower()

    if target == "robot" and not robot_id:
        print(f"[{connection_id}] Signal failed: robotId required for target=robot")
        return
    if target == "client" and not msg.get("clientConnectionId"):
        print(f"[{connection_id}] Signal failed: clientConnectionId required for target=client")
        return

    target_conn_id = None

    if target == "robot":
        presence = robot_presence.get(robot_id)
        if not presence:
            print(f"[{connection_id}] Robot {robot_id} not found/offline")
            return
        target_conn_id = presence["connectionId"]
    elif target == "client":
        target_conn_id = msg.get("clientConnectionId")

    if not target_conn_id:
        return

    target_ws = ws_map.get(target_conn_id)
    if not target_ws or not is_ws_open(target_ws):
        print(f"[{connection_id}] Target {target_conn_id} not found or not open")
        return

    dest_protocol = connection_protocol.get(target_conn_id)
    from_conn_id = connection_id

    if target == "robot":
        client_conn_id = from_conn_id
    else:
        client_conn_id = msg.get("clientConnectionId") or target_conn_id

    base_msg = {
        "type": msg_type,
        "robotId": robot_id,
        "from": from_conn_id,
        "to": target_conn_id,
        "clientConnectionId": client_conn_id,
        "payload": msg.get("payload") or {},
    }

    outbound = format_outbound(base_msg, dest_protocol)

    print(f"[{connection_id}] Forwarding {msg_type} to {target} ({target_conn_id})")
    await target_ws.send(json.dumps(outbound))
    print(f"[{connection_id}] Message forwarded successfully")


async def handle_takeover(connection_id, msg):
    robot_id = msg.get("robotId")
    if not robot_id:
        print(f"[{connection_id}] Takeover failed: robotId required")
        return

    presence = robot_presence.get(robot_id)
    if not presence:
        print(f"[{connection_id}] Robot {robot_id} not found")
        return

    robot_ws = ws_map.get(presence["connectionId"])
    if robot_ws and is_ws_open(robot_ws):
        conn = connections.get(connection_id)
        await robot_ws.send(json.dumps({
            "type": "admin-takeover",
            "robotId": robot_id,
            "by": conn["userId"] if conn else connection_id,
        }))
        print(f"[{connection_id}] Takeover message sent to robot")


async def handle_message(connection_id, msg):
    conn = connections.get(connection_id)
    if not conn:
        return

    is_new_protocol = "." in msg.get("type", "")

    if not LEGACY and not is_new_protocol:
        print(f"[{connection_id}] Legacy messages not accepted in new protocol mode")
        return

    if connection_id not in connection_protocol:
        if msg.get("type", "") not in ("ready", "ping", "pong"):
            connection_protocol[connection_id] = "modulr-v0" if is_new_protocol else "legacy"

    normalized = msg
    if is_new_protocol:
        n = normalize_new_protocol(msg)
        if not n:
            print(f"[{connection_id}] Unknown new-protocol type: {msg.get('type')}")
            return
        normalized = n

    msg_type = normalized.get("type", "")
    print(f"[{connection_id}] Received: {msg_type} {json.dumps(normalized)}")

    # ready/welcome handshake (prod webapp sends "ready" to get its connectionId)
    if msg_type == "ready":
        await send_to(connection_id, {"type": "welcome", "connectionId": connection_id})
        print(f"[{connection_id}] Sent welcome with connectionId")
        return

    if msg_type == "signalling.capabilities":
        dest_proto = connection_protocol.get(connection_id)
        if dest_proto == "modulr-v0":
            response = make_envelope("signalling.capabilities", {"versions": SUPPORTED_VERSIONS})
        else:
            response = {"type": "signalling.capabilities", "versions": SUPPORTED_VERSIONS}
        await send_to(connection_id, response)
        return

    if msg_type == "agent.ping":
        ping_id = msg.get("id") or msg.get("timestamp") or str(uuid.uuid4())
        if connection_protocol.get(connection_id) == "modulr-v0":
            response = {"type": "agent.pong", "version": "0.0", "id": f"{ping_id}-pong",
                        "timestamp": datetime.now(timezone.utc).isoformat(), "correlationId": ping_id}
        else:
            response = {"type": "agent.pong", "id": f"{ping_id}-pong", "correlationId": ping_id}
        await send_to(connection_id, response)
        return

    if msg_type == "agent.pong":
        return

    # Route by type
    if msg_type == "register":
        handle_register(connection_id, normalized)
    elif msg_type in ("offer", "answer", "ice-candidate", "candidate",
                       "signalling.connected", "signalling.disconnected", "signalling.error"):
        await handle_signal(connection_id, normalized)
    elif msg_type == "takeover":
        await handle_takeover(connection_id, normalized)
    else:
        print(f"[{connection_id}] Unknown message type: {msg_type}")


async def handler(ws, path=None):
    # Parse token from query string
    req_path = ws.request.path if hasattr(ws, 'request') else (path or "")
    parsed = urlparse(req_path)
    params = parse_qs(parsed.query)
    token = params.get("token", [None])[0]

    connection_id = f"conn-{uuid.uuid4().hex[:12]}"
    ws_map[connection_id] = ws
    ws_to_conn_id[ws] = connection_id

    user_id = token[:20] if token else connection_id
    print(f"[{connection_id}] Connected (user: {user_id[:10]}...)")

    connections[connection_id] = {
        "userId": user_id,
        "kind": "client",
        "groups": [],
    }

    try:
        async for message in ws:
            try:
                msg = json.loads(message)
                await handle_message(connection_id, msg)
            except json.JSONDecodeError as e:
                print(f"[{connection_id}] Invalid JSON: {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print(f"[{connection_id}] Disconnected")

        connections.pop(connection_id, None)
        connection_protocol.pop(connection_id, None)
        ws_map.pop(connection_id, None)
        ws_to_conn_id.pop(ws, None)

        # Remove from robot presence
        for rid in list(robot_presence.keys()):
            if robot_presence[rid]["connectionId"] == connection_id:
                del robot_presence[rid]
                print(f"[{connection_id}] Removed robot {rid} from presence")


async def main():
    local_ip = "0.0.0.0"
    proto = "LEGACY" if LEGACY else "NEW (Modulr Interface Spec)"

    print(f"Mock Signaling Server starting on ws://{local_ip}:{PORT}")
    print(f"   Protocol mode: {proto}")
    print("=====================================\n")

    async with websockets.serve(handler, local_ip, PORT, ssl=ssl_context):
        await asyncio.Future()

asyncio.run(main())
