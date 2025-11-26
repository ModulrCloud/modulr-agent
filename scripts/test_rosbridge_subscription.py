#!/usr/bin/env python3
"""
Test script to verify rosbridge can receive messages from /camera/image_raw
This helps debug if the issue is with rosbridge or the Rust code
"""

import asyncio
import websockets
import json

async def test_subscription():
    uri = "ws://localhost:9090"
    print(f"Connecting to rosbridge at {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to rosbridge!")
            
            # Subscribe to /camera/image_raw
            subscribe_msg = {
                "op": "subscribe",
                "topic": "/camera/image_raw",
                "type": "sensor_msgs/msg/Image"
            }
            print(f"Sending subscription message: {json.dumps(subscribe_msg)}")
            await websocket.send(json.dumps(subscribe_msg))
            print("Subscription sent!")
            
            # Wait for messages
            message_count = 0
            print("\nWaiting for image messages... (Press Ctrl+C to stop)")
            
            async for message in websocket:
                msg = json.loads(message)
                message_count += 1
                
                if message_count == 1:
                    print(f"\n✅ Received first message! (op: {msg.get('op')})")
                    if 'msg' in msg:
                        img_msg = msg['msg']
                        print(f"   Image info: {img_msg.get('width')}x{img_msg.get('height')}, encoding: {img_msg.get('encoding')}")
                        print(f"   Data length: {len(str(img_msg.get('data', '')))} bytes")
                
                if message_count % 10 == 0:
                    print(f"Received {message_count} messages so far...")
                    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_subscription())

