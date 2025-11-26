import asyncio, websockets, json, ssl


clients = {}
local   = True

# # TLS configuration
ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain("certs/dev-server.crt", "certs/dev-server.key")

if local:
    ssl_context = None

async def handler(ws):
    print("Waiting for connections. Connected clients: {}".format(clients))
    async for message in ws:
        msg = json.loads(message)
        print("Received message: {}".format(msg))

        # Robot registers itself
        if msg["type"] == "register":
            clients[msg["from"]] = ws
            print(f"Registered {msg['from']}")
            continue

        # Forward signaling messages
        target = clients.get(msg["to"])
        print("Found target: {}".format(target))
        if target:
            await target.send(json.dumps(msg))
    print("Connection closed")


async def main():
    local_ip = "0.0.0.0"
    port = 8765

    print(f"Starting secure WebSocket server at ws://{local_ip}:{port}")
    async with websockets.serve(handler, local_ip, port, ssl=ssl_context):
        await asyncio.Future()

asyncio.run(main())
