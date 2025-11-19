import asyncio
import websockets
import json
import ssl

SERVER_HOST = "localhost"
SERVER_PORT = 8765
URI = f"wss://{SERVER_HOST}:{SERVER_PORT}"

# SSL context (use self-signed cert)
ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
ssl_context.check_hostname = False  # Skip hostname verification (for self-signed)
ssl_context.verify_mode = ssl.CERT_NONE  # Skip certificate verification (for testing)

async def test_client():
    print(f"Connecting to wss://{SERVER_HOST}:{SERVER_PORT}...")

    try:
        async with websockets.connect(
            URI,
            ssl=ssl_context
        ) as websocket:
            print("Connected to server!")

            register_msg = {
                "type": "register",
                "from": "robot1"
            }
            await websocket.send(json.dumps(register_msg))
            print("Sent registration: robot1")

            print("Closing connection. Check logs of signaling server to verify success...")

    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Connection closed unexpectedly: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Test client finished.")


if __name__ == "__main__":
    asyncio.run(test_client())
